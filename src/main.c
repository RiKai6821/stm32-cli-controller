/**
 * @file    main.c
 * @brief   STM32 命令行控制系统
 * @details 通过串口接收文本命令，实现类似 Linux Shell 的交互体验
 *
 *   支持命令：
 *     help                 显示所有命令
 *     led on <id>          打开 LED (id: 1-3)
 *     led off <id>         关闭 LED
 *     led blink <id> <ms>  让 LED 以指定周期闪烁
 *     led status           显示所有 LED 状态
 *     pwm <id> <duty>      设置 PWM 占空比 (id: 1, duty: 0-100)
 *     uptime               显示系统运行时间
 *     reset                软复位
 *     stats                显示系统统计信息
 *
 *   硬件：STM32F103C8T6
 *     PC13: LED1 (板载，低电平点亮)
 *     PB0:  LED2
 *     PB1:  LED3
 *     PA8:  PWM 输出 (TIM1_CH1)
 *     PA9/PA10: USART1
 *
 *   也可以在 Wokwi 上验证：https://wokwi.com/projects/new/stm32
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/* ===== 配置 ===== */
#define CMD_BUF_SIZE     64
#define RESP_BUF_SIZE    256
#define LED_COUNT        3
#define UART_BAUD        115200
#define HISTORY_SIZE     8      /* 命令历史条数（环形缓冲区） */
#define PROMPT_STR       "stm32> "
#define PROMPT_LEN       7

/* LED 工作模式 */
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_BLINK
} LedMode_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    LedMode_t     mode;
    uint32_t      blink_period_ms;
    uint32_t      next_toggle;
    uint8_t       active_low;   /* 1: 低电平点亮（如 PC13） */
} Led_t;

/* ===== 全局变量 ===== */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim1;

static Led_t g_leds[LED_COUNT] = {
    { GPIOC, GPIO_PIN_13, LED_MODE_OFF, 0, 0, 1 },  /* LED1: 板载 */
    { GPIOB, GPIO_PIN_0,  LED_MODE_OFF, 0, 0, 0 },  /* LED2 */
    { GPIOB, GPIO_PIN_1,  LED_MODE_OFF, 0, 0, 0 },  /* LED3 */
};

static char cmd_buf[CMD_BUF_SIZE];
static volatile uint16_t cmd_len = 0;
static volatile uint8_t  cmd_ready = 0;
static uint8_t uart_rx_byte;

/* 系统统计 */
static uint32_t g_cmd_count = 0;
static uint32_t g_cmd_err_count = 0;
static uint32_t g_start_tick;

/* ===== 命令历史（环形缓冲区） ===== */
static char    g_history[HISTORY_SIZE][CMD_BUF_SIZE];
static uint8_t g_hist_head  = 0;   /* 下次写入位置 */
static uint8_t g_hist_count = 0;   /* 已保存条数 */
static int8_t  g_hist_pos   = -1;  /* 浏览游标，-1 = 非历史模式 */

/* ISR 内 VT100 ESC 序列状态机：0 正常 / 1 收到 ESC / 2 收到 ESC[ */
static volatile uint8_t g_esc_state = 0;

/* ===== 前向声明 ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

static void uart_print(const char *fmt, ...);
static void process_command(char *cmd);
static void led_set(uint8_t id, uint8_t on);
static void update_leds(void);
static void show_prompt(void);

/* ===== 命令处理函数声明 ===== */
static void cmd_help(int argc, char *argv[]);
static void cmd_led(int argc, char *argv[]);
static void cmd_pwm(int argc, char *argv[]);
static void cmd_uptime(int argc, char *argv[]);
static void cmd_reset(int argc, char *argv[]);
static void cmd_stats(int argc, char *argv[]);
static void cmd_history(int argc, char *argv[]);

/* ===== 命令历史辅助函数 ===== */
static void       history_push(const char *s, uint16_t len);
static const char *history_get(int8_t offset);
static void       isr_replace_line(const char *s, uint16_t len);

/* 命令表（函数指针数组，可扩展） */
typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} Command_t;

static const Command_t cmd_table[] = {
    { "help",   cmd_help,   "Show all commands" },
    { "led",    cmd_led,    "led <on|off|blink|status> <id> [param]" },
    { "pwm",    cmd_pwm,    "pwm <id> <duty 0-100>" },
    { "uptime", cmd_uptime, "Show system uptime" },
    { "stats",   cmd_stats,   "Show command statistics" },
    { "history", cmd_history, "Show command history (or use ↑↓ arrows)" },
    { "reset",   cmd_reset,   "Soft reset" },
    { NULL,      NULL,        NULL }
};

/* ===== main ===== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    /* 初始化 LED 关闭 */
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, 0);
    }
    
    g_start_tick = HAL_GetTick();
    
    /* 启动 UART 中断接收 */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    
    /* 欢迎信息 */
    uart_print("\r\n");
    uart_print("=====================================\r\n");
    uart_print(" STM32 Command Line Controller v1.0\r\n");
    uart_print(" Type 'help' for command list\r\n");
    uart_print("=====================================\r\n");
    show_prompt();
    
    while (1) {
        /* 命令解析 */
        if (cmd_ready) {
            process_command(cmd_buf);
            cmd_len = 0;
            cmd_ready = 0;
            show_prompt();
        }
        
        /* LED 状态机更新 */
        update_leds();
    }
}

/* ===== UART ===== */
static void uart_print(const char *fmt, ...)
{
    static char buf[RESP_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
    }
}

static void show_prompt(void)
{
    uart_print("stm32> ");
}

/* UART 中断回调：收一个字节就处理，支持 VT100 方向键历史浏览 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    /* 上一条命令尚未处理完，不写缓冲区 */
    if (cmd_ready) {
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        return;
    }

    uint8_t b = uart_rx_byte;

    /* ---- VT100 ESC 序列状态机 ---- */
    if (g_esc_state == 1) {
        g_esc_state = (b == '[') ? 2 : 0;
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        return;
    }
    if (g_esc_state == 2) {
        g_esc_state = 0;
        if (b == 'A') {
            /* ↑ 上箭头：往更早的历史移动 */
            int8_t next = g_hist_pos + 1;
            const char *h = history_get(next);
            if (h) {
                g_hist_pos = next;
                uint16_t hlen = (uint16_t)strlen(h);
                memcpy(cmd_buf, h, hlen + 1);
                cmd_len = hlen;
                isr_replace_line(h, hlen);
            }
        } else if (b == 'B') {
            /* ↓ 下箭头：往更新的历史移动 */
            int8_t next = g_hist_pos - 1;
            g_hist_pos = next;
            if (next < 0) {
                /* 回到新输入 */
                cmd_len = 0;
                cmd_buf[0] = '\0';
                isr_replace_line("", 0);
            } else {
                const char *h = history_get(next);
                if (h) {
                    uint16_t hlen = (uint16_t)strlen(h);
                    memcpy(cmd_buf, h, hlen + 1);
                    cmd_len = hlen;
                    isr_replace_line(h, hlen);
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        return;
    }

    /* ---- 普通字符处理 ---- */
    if (b == 0x1B) {
        /* ESC 起始 */
        g_esc_state = 1;
    } else if (b == 0x03) {
        /* Ctrl+C：清空当前输入 */
        cmd_len = 0;
        cmd_buf[0] = '\0';
        g_hist_pos = -1;
        HAL_UART_Transmit(&huart1, (uint8_t *)"^C\r\n" PROMPT_STR, 4 + PROMPT_LEN, 20);
    } else if (b == '\r' || b == '\n') {
        if (cmd_len > 0) {
            cmd_buf[cmd_len] = '\0';
            history_push(cmd_buf, cmd_len);
            cmd_ready = 1;
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 10);
    } else if (b == 0x08 || b == 0x7F) {
        /* 退格 */
        if (cmd_len > 0) {
            cmd_len--;
            HAL_UART_Transmit(&huart1, (uint8_t *)"\b \b", 3, 10);
        }
    } else if (cmd_len < CMD_BUF_SIZE - 1 && b >= 0x20 && b < 0x7F) {
        /* 可打印字符：退出历史浏览模式 */
        g_hist_pos = -1;
        cmd_buf[cmd_len++] = b;
        HAL_UART_Transmit(&huart1, &b, 1, 10);
    }

    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* ===== 命令处理 ===== */

/* 简单的命令分词器 */
static int tokenize(char *cmd, char *argv[], int max_args)
{
    int argc = 0;
    char *p = cmd;
    while (*p && argc < max_args) {
        /* 跳过空白 */
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        /* 找下一个空白 */
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) {
            *p = '\0';
            p++;
        }
    }
    return argc;
}

static void process_command(char *cmd)
{
    char *argv[8];
    int argc = tokenize(cmd, argv, 8);
    
    if (argc == 0) return;
    
    g_cmd_count++;
    
    /* 在命令表中查找 */
    for (const Command_t *c = cmd_table; c->name != NULL; c++) {
        if (strcmp(argv[0], c->name) == 0) {
            c->handler(argc, argv);
            return;
        }
    }
    
    g_cmd_err_count++;
    uart_print("Error: unknown command '%s'. Type 'help'.\r\n", argv[0]);
}

/* ===== 命令实现 ===== */

static void cmd_help(int argc, char *argv[])
{
    uart_print("Available commands:\r\n");
    for (const Command_t *c = cmd_table; c->name != NULL; c++) {
        uart_print("  %-8s - %s\r\n", c->name, c->help);
    }
}

static void cmd_led(int argc, char *argv[])
{
    if (argc < 2) {
        uart_print("Usage: led <on|off|blink|status> <id> [period_ms]\r\n");
        g_cmd_err_count++;
        return;
    }
    
    if (strcmp(argv[1], "status") == 0) {
        const char *mode_str[] = { "OFF", "ON", "BLINK" };
        for (int i = 0; i < LED_COUNT; i++) {
            uart_print("  LED%d: %s", i + 1, mode_str[g_leds[i].mode]);
            if (g_leds[i].mode == LED_MODE_BLINK) {
                uart_print(" (%lu ms)", g_leds[i].blink_period_ms);
            }
            uart_print("\r\n");
        }
        return;
    }
    
    if (argc < 3) {
        uart_print("Error: missing LED id\r\n");
        g_cmd_err_count++;
        return;
    }
    
    int id = atoi(argv[2]) - 1;
    if (id < 0 || id >= LED_COUNT) {
        uart_print("Error: LED id must be 1-%d\r\n", LED_COUNT);
        g_cmd_err_count++;
        return;
    }
    
    if (strcmp(argv[1], "on") == 0) {
        g_leds[id].mode = LED_MODE_ON;
        led_set(id, 1);
        uart_print("LED%d ON\r\n", id + 1);
    } else if (strcmp(argv[1], "off") == 0) {
        g_leds[id].mode = LED_MODE_OFF;
        led_set(id, 0);
        uart_print("LED%d OFF\r\n", id + 1);
    } else if (strcmp(argv[1], "blink") == 0) {
        if (argc < 4) {
            uart_print("Usage: led blink <id> <period_ms>\r\n");
            g_cmd_err_count++;
            return;
        }
        int period = atoi(argv[3]);
        if (period < 10 || period > 60000) {
            uart_print("Error: period must be 10-60000 ms\r\n");
            g_cmd_err_count++;
            return;
        }
        g_leds[id].mode = LED_MODE_BLINK;
        g_leds[id].blink_period_ms = period;
        g_leds[id].next_toggle = HAL_GetTick() + period;
        uart_print("LED%d BLINK %d ms\r\n", id + 1, period);
    } else {
        uart_print("Error: unknown LED action '%s'\r\n", argv[1]);
        g_cmd_err_count++;
    }
}

static void cmd_pwm(int argc, char *argv[])
{
    if (argc < 3) {
        uart_print("Usage: pwm <id> <duty 0-100>\r\n");
        g_cmd_err_count++;
        return;
    }
    
    int id = atoi(argv[1]);
    int duty = atoi(argv[2]);
    
    if (id != 1) {
        uart_print("Error: only PWM 1 is supported\r\n");
        g_cmd_err_count++;
        return;
    }
    if (duty < 0 || duty > 100) {
        uart_print("Error: duty must be 0-100\r\n");
        g_cmd_err_count++;
        return;
    }
    
    /* 假设 ARR = 999，CCR 范围 0-1000 */
    uint32_t ccr = (htim1.Init.Period + 1) * duty / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    uart_print("PWM%d duty = %d%%\r\n", id, duty);
}

static void cmd_uptime(int argc, char *argv[])
{
    uint32_t ms = HAL_GetTick() - g_start_tick;
    uint32_t s = ms / 1000;
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
    HAL_Delay(100);
    NVIC_SystemReset();
}

/* ===== 命令历史 ===== */

/* 保存命令到历史环形缓冲区（在 ISR 中调用） */
static void history_push(const char *s, uint16_t len)
{
    if (len == 0) return;
    memcpy(g_history[g_hist_head], s, len + 1);
    g_hist_head = (g_hist_head + 1) % HISTORY_SIZE;
    if (g_hist_count < HISTORY_SIZE) g_hist_count++;
    g_hist_pos = -1;
}

/* 获取历史条目（offset=0 最近，offset=1 更早）；在 ISR 或 main 中均可调用 */
static const char *history_get(int8_t offset)
{
    if (offset < 0 || offset >= (int8_t)g_hist_count) return NULL;
    uint8_t idx = (g_hist_head - 1 - (uint8_t)offset + HISTORY_SIZE * 2) % HISTORY_SIZE;
    return g_history[idx];
}

/* 在 ISR 中：用 VT100 清除当前行并重新显示提示符 + 新内容 */
static void isr_replace_line(const char *s, uint16_t len)
{
    /* \r 回到行首，\x1b[2K 擦除整行，然后重新打印提示符 */
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\x1b[2K" PROMPT_STR, 5 + PROMPT_LEN, 20);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)s, len, 50);
    }
}

static void cmd_history(int argc, char *argv[])
{
    /* 关中断快速读取游标，防止 ISR 并发修改 */
    __disable_irq();
    uint8_t cnt  = g_hist_count;
    uint8_t head = g_hist_head;
    __enable_irq();

    if (cnt == 0) {
        uart_print("  (no history)\r\n");
        return;
    }
    for (int8_t i = (int8_t)cnt - 1; i >= 0; i--) {
        uint8_t idx = (head - 1 - (uint8_t)i + HISTORY_SIZE * 2) % HISTORY_SIZE;
        uart_print("  %3d  %s\r\n", (int)(cnt - 1 - (uint8_t)i), g_history[idx]);
    }
}

/* ===== LED 控制 ===== */

static void led_set(uint8_t id, uint8_t on)
{
    if (id >= LED_COUNT) return;
    GPIO_PinState state = (on ^ g_leds[id].active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(g_leds[id].port, g_leds[id].pin, state);
}

static void update_leds(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < LED_COUNT; i++) {
        if (g_leds[i].mode == LED_MODE_BLINK && now >= g_leds[i].next_toggle) {
            HAL_GPIO_TogglePin(g_leds[i].port, g_leds[i].pin);
            g_leds[i].next_toggle = now + g_leds[i].blink_period_ms;
        }
    }
}

/* ===== 错误处理 ===== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
