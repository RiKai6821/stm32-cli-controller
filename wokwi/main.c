/**
 * @file    main.c  (Wokwi — STM32 Nucleo-64 C031C6, Arduino 框架)
 * @version 1.1
 *
 * 功能：
 *   - 8 条 CLI 命令（help / led / pwm / uptime / sysinfo / stats / history / reset）
 *   - LED 状态机：on / off / blink（可配置周期，非阻塞）
 *   - PWM 渐变状态机（呼吸灯）：pwm fade <step> <ms>，非阻塞，不干扰串口
 *   - VT100 命令历史：↑↓ 方向键翻 8 条历史，Ctrl+C 清行
 *   - sysinfo：实时显示 MCU 型号 / 时钟 / 引脚图 / 固件版本 / 运行时状态
 *
 * 为什么不用裸 HAL？
 *   Wokwi C031C6 使用 Arduino STM32 框架编译，SrcWrapper 库已定义了
 *   HAL_TIM_Base_MspInit / HAL_UART_RxCpltCallback 等函数，重复定义
 *   会导致 "multiple definition" 链接错误。这里改用 Arduino API：
 *     Serial        替代 HAL UART
 *     digitalWrite  替代 HAL GPIO
 *     analogWrite   替代 HAL TIM PWM
 *     millis()      替代 HAL_GetTick()
 *   命令解析、LED 状态机、历史记录等核心逻辑完全不变。
 *
 * 引脚：
 *   PA2/PA3 - Serial Monitor（diagram.json 已连线）
 *   PA5     - LED1 板载 LD2            led on/off/blink 1
 *   PA8     - PWM 输出（黄色LED）      pwm 1 <0-100> | pwm fade <step> <ms>
 *   PB0     - LED2 绿色                led on/off/blink 2
 *   PB1     - LED3 蓝色                led on/off/blink 3
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ===== 配置 ===== */
#define CMD_BUF_SIZE  64
#define RESP_BUF_SIZE 256
#define LED_COUNT     3
#define HISTORY_SIZE  8
#define PROMPT_STR    "stm32> "
#define PROMPT_LEN    7

typedef enum { LED_MODE_OFF = 0, LED_MODE_ON, LED_MODE_BLINK } LedMode_t;

typedef struct {
    int       pin;               /* Arduino 引脚号：PA5 / PB0 / PB1 */
    LedMode_t mode;
    uint32_t  blink_period_ms;
    uint32_t  next_toggle;
} Led_t;

/* ===== 全局变量 ===== */
static Led_t g_leds[LED_COUNT] = {
    { PA5, LED_MODE_OFF, 0, 0 },   /* LED1: 板载 LD2 */
    { PB0, LED_MODE_OFF, 0, 0 },   /* LED2: 绿色 */
    { PB1, LED_MODE_OFF, 0, 0 },   /* LED3: 蓝色 */
};

static char    cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len   = 0;
static uint8_t  cmd_ready = 0;

static uint32_t g_cmd_count     = 0;
static uint32_t g_cmd_err_count = 0;
static uint32_t g_start_tick;

static char    g_history[HISTORY_SIZE][CMD_BUF_SIZE];
static uint8_t g_hist_head  = 0;
static uint8_t g_hist_count = 0;
static int8_t  g_hist_pos   = -1;
static uint8_t g_esc_state  = 0;

/* ===== PWM 渐变状态机 ===== */
typedef struct {
    uint8_t  active;       /* 1 = 正在渐变 */
    int8_t   direction;    /* +1=亮 -1=暗 */
    uint8_t  step;         /* 每次步进 duty% */
    uint32_t interval_ms;  /* 步进间隔 ms */
    uint32_t next_ms;      /* 下次变化时间 */
    int16_t  current;      /* 当前 duty (0-100) */
} PwmFade_t;

static PwmFade_t g_fade = { 0 };

/* ===== 前向声明 ===== */
static void uart_print(const char *fmt, ...);
static void show_prompt(void);
static void handle_byte(uint8_t b);
static void process_command(char *cmd);
static void led_set(uint8_t id, uint8_t on);
static void update_leds(void);
static void update_pwm_fade(void);
static void cmd_help(int argc, char *argv[]);
static void cmd_led(int argc, char *argv[]);
static void cmd_pwm(int argc, char *argv[]);
static void cmd_uptime(int argc, char *argv[]);
static void cmd_reset(int argc, char *argv[]);
static void cmd_stats(int argc, char *argv[]);
static void cmd_history_cmd(int argc, char *argv[]);
static void cmd_sysinfo(int argc, char *argv[]);
static void       history_push(const char *s, uint16_t len);
static const char *history_get(int8_t offset);
static void       replace_line(const char *s, uint16_t len);

typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} Command_t;

static const Command_t cmd_table[] = {
    { "help",    cmd_help,        "Show all commands" },
    { "led",     cmd_led,         "led <on|off|blink|status> <id> [ms]" },
    { "pwm",     cmd_pwm,         "pwm <id> <duty 0-100> | pwm fade <step> <ms> | pwm stop" },
    { "uptime",  cmd_uptime,      "Show system uptime" },
    { "sysinfo", cmd_sysinfo,     "Show MCU info and pin map" },
    { "stats",   cmd_stats,       "Show command statistics" },
    { "history", cmd_history_cmd, "Show command history (up/down arrows)" },
    { "reset",   cmd_reset,       "Soft reset" },
    { NULL, NULL, NULL }
};

/* ===== Arduino 入口 ===== */
void setup(void)
{
    Serial.begin(115200);

    pinMode(PA5, OUTPUT); digitalWrite(PA5, LOW);
    pinMode(PB0, OUTPUT); digitalWrite(PB0, LOW);
    pinMode(PB1, OUTPUT); digitalWrite(PB1, LOW);
    pinMode(PA8, OUTPUT); analogWrite(PA8, 0);

    g_start_tick = millis();

    uart_print("\r\n");
    uart_print("=====================================\r\n");
    uart_print(" STM32 Command Line Controller v1.1\r\n");
    uart_print(" Type 'help' for command list\r\n");
    uart_print("=====================================\r\n");
    show_prompt();
}

void loop(void)
{
    /* 轮询读取串口输入 */
    while (Serial.available()) {
        handle_byte((uint8_t)Serial.read());
    }

    if (cmd_ready) {
        process_command(cmd_buf);
        cmd_len   = 0;
        cmd_ready = 0;
        show_prompt();
    }

    update_leds();
    update_pwm_fade();
}

/* ===== UART 工具（封装 Arduino Serial） ===== */
static void uart_print(const char *fmt, ...)
{
    static char buf[RESP_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) Serial.write((const uint8_t *)buf, len);
}

static void show_prompt(void)
{
    Serial.write(PROMPT_STR);
}

/* ===== 字节处理（原 UART ISR 逻辑，现在在 loop 里轮询调用） ===== */
static void handle_byte(uint8_t b)
{
    if (cmd_ready) return;

    /* VT100 ESC 序列状态机 */
    if (g_esc_state == 1) {
        g_esc_state = (b == '[') ? 2 : 0;
        return;
    }
    if (g_esc_state == 2) {
        g_esc_state = 0;
        if (b == 'A') {                            /* ↑ 上翻历史 */
            int8_t next = g_hist_pos + 1;
            const char *h = history_get(next);
            if (h) {
                g_hist_pos = next;
                uint16_t hlen = (uint16_t)strlen(h);
                memcpy(cmd_buf, h, hlen + 1);
                cmd_len = hlen;
                replace_line(h, hlen);
            }
        } else if (b == 'B') {                     /* ↓ 下翻历史 */
            int8_t next = g_hist_pos - 1;
            g_hist_pos = next;
            if (next < 0) {
                cmd_len = 0; cmd_buf[0] = '\0';
                replace_line("", 0);
            } else {
                const char *h = history_get(next);
                if (h) {
                    uint16_t hlen = (uint16_t)strlen(h);
                    memcpy(cmd_buf, h, hlen + 1);
                    cmd_len = hlen;
                    replace_line(h, hlen);
                }
            }
        }
        return;
    }

    if (b == 0x1B) {
        g_esc_state = 1;
    } else if (b == 0x03) {                        /* Ctrl+C 清行 */
        cmd_len = 0; cmd_buf[0] = '\0'; g_hist_pos = -1;
        Serial.write("^C\r\n" PROMPT_STR);
    } else if (b == '\r' || b == '\n') {
        if (cmd_len > 0) {
            cmd_buf[cmd_len] = '\0';
            history_push(cmd_buf, cmd_len);
            cmd_ready = 1;
        }
        Serial.write("\r\n");
    } else if (b == 0x08 || b == 0x7F) {          /* 退格 */
        if (cmd_len > 0) {
            cmd_len--;
            Serial.write("\b \b");
        }
    } else if (cmd_len < CMD_BUF_SIZE - 1 && b >= 0x20 && b < 0x7F) {
        g_hist_pos = -1;
        cmd_buf[cmd_len++] = b;
        Serial.write(b);
    }
}

/* ===== 命令解析 ===== */
static int tokenize(char *cmd, char *argv[], int max_args)
{
    int argc = 0; char *p = cmd;
    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) { *p = '\0'; p++; }
    }
    return argc;
}

static void process_command(char *cmd)
{
    char *argv[8];
    int argc = tokenize(cmd, argv, 8);
    if (argc == 0) return;
    g_cmd_count++;
    for (const Command_t *c = cmd_table; c->name; c++) {
        if (strcmp(argv[0], c->name) == 0) { c->handler(argc, argv); return; }
    }
    g_cmd_err_count++;
    uart_print("Error: unknown command '%s'. Type 'help'.\r\n", argv[0]);
}

/* ===== 命令实现 ===== */
static void cmd_help(int argc, char *argv[])
{
    uart_print("Available commands:\r\n");
    for (const Command_t *c = cmd_table; c->name; c++)
        uart_print("  %-8s - %s\r\n", c->name, c->help);
}

static void cmd_led(int argc, char *argv[])
{
    if (argc < 2) { uart_print("Usage: led <on|off|blink|status> <id> [ms]\r\n"); return; }
    if (strcmp(argv[1], "status") == 0) {
        const char *m[] = { "OFF", "ON", "BLINK" };
        for (int i = 0; i < LED_COUNT; i++) {
            uart_print("  LED%d: %s", i + 1, m[g_leds[i].mode]);
            if (g_leds[i].mode == LED_MODE_BLINK)
                uart_print(" (%lu ms)", g_leds[i].blink_period_ms);
            uart_print("\r\n");
        }
        return;
    }
    if (argc < 3) { uart_print("Error: missing LED id\r\n"); return; }
    int id = atoi(argv[2]) - 1;
    if (id < 0 || id >= LED_COUNT) {
        uart_print("Error: LED id must be 1-%d\r\n", LED_COUNT); return;
    }
    if (strcmp(argv[1], "on") == 0) {
        g_leds[id].mode = LED_MODE_ON; led_set(id, 1);
        uart_print("LED%d ON\r\n", id + 1);
    } else if (strcmp(argv[1], "off") == 0) {
        g_leds[id].mode = LED_MODE_OFF; led_set(id, 0);
        uart_print("LED%d OFF\r\n", id + 1);
    } else if (strcmp(argv[1], "blink") == 0) {
        if (argc < 4) { uart_print("Usage: led blink <id> <period_ms>\r\n"); return; }
        int period = atoi(argv[3]);
        if (period < 10 || period > 60000) {
            uart_print("Error: period must be 10-60000 ms\r\n"); return;
        }
        g_leds[id].mode            = LED_MODE_BLINK;
        g_leds[id].blink_period_ms = period;
        g_leds[id].next_toggle     = millis() + period;
        uart_print("LED%d BLINK %d ms\r\n", id + 1, period);
    } else {
        uart_print("Error: unknown action '%s'\r\n", argv[1]);
    }
}

static void cmd_pwm(int argc, char *argv[])
{
    if (argc < 2) {
        uart_print("Usage: pwm <id> <duty 0-100>\r\n");
        uart_print("       pwm fade <step 1-20> <interval_ms>\r\n");
        uart_print("       pwm stop\r\n");
        return;
    }

    /* pwm fade <step> <interval_ms> — 启动非阻塞呼吸灯渐变 */
    if (strcmp(argv[1], "fade") == 0) {
        if (argc < 4) {
            uart_print("Usage: pwm fade <step 1-20> <interval_ms 10-2000>\r\n");
            return;
        }
        int step = atoi(argv[2]);
        int ms   = atoi(argv[3]);
        if (step < 1 || step > 20) { uart_print("Error: step must be 1-20\r\n");       return; }
        if (ms < 10 || ms > 2000)  { uart_print("Error: interval must be 10-2000 ms\r\n"); return; }
        g_fade.step        = (uint8_t)step;
        g_fade.interval_ms = (uint32_t)ms;
        g_fade.direction   = +1;
        g_fade.current     = 0;
        g_fade.next_ms     = millis() + (uint32_t)ms;
        g_fade.active      = 1;
        uart_print("PWM fade ON: step=%d%% every %d ms  (0->100->0 loop)\r\n", step, ms);
        return;
    }

    /* pwm stop — 停止渐变，关闭输出 */
    if (strcmp(argv[1], "stop") == 0) {
        g_fade.active = 0;
        analogWrite(PA8, 0);
        uart_print("PWM fade stopped, PA8 = 0%%\r\n");
        return;
    }

    /* pwm <id> <duty> — 设置固定占空比（同时停止渐变） */
    if (argc < 3) { uart_print("Usage: pwm <id> <duty 0-100>\r\n"); return; }
    int id = atoi(argv[1]), duty = atoi(argv[2]);
    if (id != 1)                 { uart_print("Error: only PWM id 1 (PA8) supported\r\n"); return; }
    if (duty < 0 || duty > 100)  { uart_print("Error: duty must be 0-100\r\n");            return; }
    g_fade.active = 0;                              /* 固定占空比时关闭渐变 */
    analogWrite(PA8, (int)(duty * 255 / 100));      /* analogWrite 范围 0-255 */
    uart_print("PWM1 (PA8) duty = %d%%\r\n", duty);
}

static void cmd_uptime(int argc, char *argv[])
{
    uint32_t ms = millis() - g_start_tick, s = ms / 1000;
    uart_print("Uptime: %lu.%03lu s  (%lu:%02lu:%02lu)\r\n",
               s, ms % 1000, s / 3600, (s / 60) % 60, s % 60);
}

static void cmd_stats(int argc, char *argv[])
{
    uart_print("System statistics:\r\n");
    uart_print("  Commands processed: %lu\r\n", g_cmd_count);
    uart_print("  Command errors:     %lu\r\n", g_cmd_err_count);
    uart_print("  Success rate:       %lu%%\r\n",
               g_cmd_count > 0 ? (g_cmd_count - g_cmd_err_count) * 100 / g_cmd_count : 0);
}

static void cmd_reset(int argc, char *argv[])
{
    uart_print("Resetting...\r\n");
    delay(100);
    NVIC_SystemReset();
}

/* ===== 命令历史 ===== */
static void history_push(const char *s, uint16_t len)
{
    if (len == 0) return;
    memcpy(g_history[g_hist_head], s, len + 1);
    g_hist_head = (g_hist_head + 1) % HISTORY_SIZE;
    if (g_hist_count < HISTORY_SIZE) g_hist_count++;
    g_hist_pos = -1;
}

static const char *history_get(int8_t offset)
{
    if (offset < 0 || offset >= (int8_t)g_hist_count) return NULL;
    uint8_t idx = (g_hist_head - 1 - (uint8_t)offset + HISTORY_SIZE * 2) % HISTORY_SIZE;
    return g_history[idx];
}

/* VT100：清除当前行并重新显示提示符 + 新内容 */
static void replace_line(const char *s, uint16_t len)
{
    Serial.write("\r\x1b[2K" PROMPT_STR);
    if (len > 0) Serial.write((const uint8_t *)s, len);
}

static void cmd_history_cmd(int argc, char *argv[])
{
    if (g_hist_count == 0) { uart_print("  (no history)\r\n"); return; }
    for (int8_t i = (int8_t)g_hist_count - 1; i >= 0; i--) {
        uint8_t idx = (g_hist_head - 1 - (uint8_t)i + HISTORY_SIZE * 2) % HISTORY_SIZE;
        uart_print("  %3d  %s\r\n", (int)(g_hist_count - 1 - (uint8_t)i), g_history[idx]);
    }
}

/* ===== LED 控制 ===== */
static void led_set(uint8_t id, uint8_t on)
{
    if (id >= LED_COUNT) return;
    digitalWrite(g_leds[id].pin, on ? HIGH : LOW);
}

static void update_leds(void)
{
    uint32_t now = millis();
    for (int i = 0; i < LED_COUNT; i++) {
        if (g_leds[i].mode == LED_MODE_BLINK && now >= g_leds[i].next_toggle) {
            digitalWrite(g_leds[i].pin, !digitalRead(g_leds[i].pin));
            g_leds[i].next_toggle = now + g_leds[i].blink_period_ms;
        }
    }
}

/* ===== PWM 渐变状态机（非阻塞，在 loop() 中调用） =====
 *
 * 原理：每隔 interval_ms 毫秒将 PA8 占空比增加或减少 step%。
 * 到达 100% 时方向反转（开始减暗），到达 0% 时再次反转，
 * 形成连续的"呼吸灯"效果——无 delay()，不阻塞串口接收。
 */
static void update_pwm_fade(void)
{
    if (!g_fade.active) return;

    uint32_t now = millis();
    if (now < g_fade.next_ms) return;          /* 未到步进时刻，直接返回 */

    g_fade.next_ms = now + g_fade.interval_ms;
    g_fade.current += g_fade.direction * (int16_t)g_fade.step;

    if (g_fade.current >= 100) {
        g_fade.current   = 100;
        g_fade.direction = -1;                 /* 顶端：转为减暗 */
    } else if (g_fade.current <= 0) {
        g_fade.current   = 0;
        g_fade.direction = +1;                 /* 底端：转为加亮 */
    }

    analogWrite(PA8, (int)(g_fade.current * 255 / 100));
}

/* ===== sysinfo 命令：显示 MCU 参数、引脚图、固件版本 ===== */
static void cmd_sysinfo(int argc, char *argv[])
{
    uint32_t ms   = millis() - g_start_tick;
    uint32_t upss = ms / 1000;

    uart_print("=== System Information ===\r\n");
    uart_print("  MCU       : STM32C031C6  (Cortex-M0+)\r\n");
    uart_print("  Clock     : 48 MHz  (HSI, no PLL)\r\n");
    uart_print("  Flash     : 32 KB    RAM: 12 KB\r\n");
    uart_print("  Framework : Arduino STM32 (SrcWrapper)\r\n");
    uart_print("  Firmware  : CLI-Controller v1.1\r\n");
    uart_print("  Build     : " __DATE__ " " __TIME__ "\r\n");
    uart_print("\r\n");
    uart_print("=== Pin Map ===\r\n");
    uart_print("  PA2  TX   -> Serial Monitor (115200 baud)\r\n");
    uart_print("  PA3  RX   -> Serial Monitor\r\n");
    uart_print("  PA5  OUT  -> LED1 (board LD2, active-high)\r\n");
    uart_print("  PA8  PWM  -> LED4 yellow  [pwm 1 <duty>]\r\n");
    uart_print("  PB0  OUT  -> LED2 green\r\n");
    uart_print("  PB1  OUT  -> LED3 blue\r\n");
    uart_print("\r\n");
    uart_print("=== Runtime ===\r\n");
    uart_print("  Uptime    : %lu:%02lu:%02lu\r\n",
               upss / 3600, (upss / 60) % 60, upss % 60);
    uart_print("  Tick      : %lu ms\r\n", ms);
    uart_print("  PWM fade  : %s", g_fade.active ? "RUNNING" : "idle");
    if (g_fade.active)
        uart_print("  duty=%d%%  step=%d%%  interval=%lu ms",
                   (int)g_fade.current, (int)g_fade.step, g_fade.interval_ms);
    uart_print("\r\n");
}
