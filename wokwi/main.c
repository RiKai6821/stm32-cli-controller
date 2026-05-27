/**
 * @file    main.c  (Wokwi 版 — STM32 Nucleo-64 C031C6)
 * @brief   STM32 命令行控制系统，适配 Wokwi C031C6 仿真
 *
 * 使用方法：
 *   1. 在 Wokwi 点击 "STM32 Nucleo-64 C031C6" 新建项目
 *   2. 将本文件内容粘贴到 main.c
 *   3. 将 diagram.json 内容粘贴到 diagram.json 标签
 *   4. 点击 ▶ 运行，在 Serial Monitor 输入命令
 *
 * 引脚映射（C031C6 Nucleo-64）：
 *   PA5  - LED1（板载 LD2，高电平点亮）
 *   PB0  - LED2（绿色）
 *   PB1  - LED3（蓝色）
 *   PA8  - TIM1_CH1 PWM（黄色 LED，亮度随占空比变化）
 *   PA9  - USART1_TX → Serial Monitor
 *   PA10 - USART1_RX ← Serial Monitor
 */

#include "stm32c0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/* ===== 配置 ===== */
#define CMD_BUF_SIZE     64
#define RESP_BUF_SIZE    256
#define LED_COUNT        3
#define UART_BAUD        115200
#define HISTORY_SIZE     8
#define PROMPT_STR       "stm32> "
#define PROMPT_LEN       7

typedef enum { LED_MODE_OFF = 0, LED_MODE_ON, LED_MODE_BLINK } LedMode_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    LedMode_t     mode;
    uint32_t      blink_period_ms;
    uint32_t      next_toggle;
    uint8_t       active_low;
} Led_t;

/* ===== 全局变量 ===== */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim1;

/* C031C6 Nucleo-64: LED1 = PA5 (LD2, 高电平亮), LED2/3 = PB0/PB1 */
static Led_t g_leds[LED_COUNT] = {
    { GPIOA, GPIO_PIN_5, LED_MODE_OFF, 0, 0, 0 },
    { GPIOB, GPIO_PIN_0, LED_MODE_OFF, 0, 0, 0 },
    { GPIOB, GPIO_PIN_1, LED_MODE_OFF, 0, 0, 0 },
};

static char cmd_buf[CMD_BUF_SIZE];
static volatile uint16_t cmd_len   = 0;
static volatile uint8_t  cmd_ready = 0;
static uint8_t uart_rx_byte;

static uint32_t g_cmd_count     = 0;
static uint32_t g_cmd_err_count = 0;
static uint32_t g_start_tick;

static char    g_history[HISTORY_SIZE][CMD_BUF_SIZE];
static uint8_t g_hist_head  = 0;
static uint8_t g_hist_count = 0;
static int8_t  g_hist_pos   = -1;
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
static void cmd_help(int argc, char *argv[]);
static void cmd_led(int argc, char *argv[]);
static void cmd_pwm(int argc, char *argv[]);
static void cmd_uptime(int argc, char *argv[]);
static void cmd_reset(int argc, char *argv[]);
static void cmd_stats(int argc, char *argv[]);
static void cmd_history(int argc, char *argv[]);
static void       history_push(const char *s, uint16_t len);
static const char *history_get(int8_t offset);
static void       isr_replace_line(const char *s, uint16_t len);

typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} Command_t;

static const Command_t cmd_table[] = {
    { "help",    cmd_help,    "Show all commands" },
    { "led",     cmd_led,     "led <on|off|blink|status> <id> [ms]" },
    { "pwm",     cmd_pwm,     "pwm <id> <duty 0-100>" },
    { "uptime",  cmd_uptime,  "Show system uptime" },
    { "stats",   cmd_stats,   "Show command statistics" },
    { "history", cmd_history, "Show command history (up/down arrows)" },
    { "reset",   cmd_reset,   "Soft reset" },
    { NULL, NULL, NULL }
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

    for (int i = 0; i < LED_COUNT; i++) led_set(i, 0);

    g_start_tick = HAL_GetTick();
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

    uart_print("\r\n");
    uart_print("=====================================\r\n");
    uart_print(" STM32 Command Line Controller v1.0\r\n");
    uart_print(" Type 'help' for command list\r\n");
    uart_print("=====================================\r\n");
    show_prompt();

    while (1) {
        if (cmd_ready) {
            process_command(cmd_buf);
            cmd_len = 0;
            cmd_ready = 0;
            show_prompt();
        }
        update_leds();
    }
}

/* ===== HAL MSP：USART1 引脚 + 中断 ===== */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_9;         /* TX */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_10;        /* RX */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

/* ===== HAL MSP：TIM1 时钟 ===== */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

/* ===== HAL MSP 后处理：TIM1_CH1 → PA8 ===== */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ===== USART1 中断入口 ===== */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* ===== UART 接收回调 ===== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (cmd_ready) {
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        return;
    }

    uint8_t b = uart_rx_byte;

    if (g_esc_state == 1) {
        g_esc_state = (b == '[') ? 2 : 0;
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
        return;
    }
    if (g_esc_state == 2) {
        g_esc_state = 0;
        if (b == 'A') {
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
            int8_t next = g_hist_pos - 1;
            g_hist_pos = next;
            if (next < 0) {
                cmd_len = 0; cmd_buf[0] = '\0';
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

    if (b == 0x1B) {
        g_esc_state = 1;
    } else if (b == 0x03) {
        cmd_len = 0; cmd_buf[0] = '\0'; g_hist_pos = -1;
        HAL_UART_Transmit(&huart1, (uint8_t *)"^C\r\n" PROMPT_STR, 4 + PROMPT_LEN, 20);
    } else if (b == '\r' || b == '\n') {
        if (cmd_len > 0) {
            cmd_buf[cmd_len] = '\0';
            history_push(cmd_buf, cmd_len);
            cmd_ready = 1;
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 10);
    } else if (b == 0x08 || b == 0x7F) {
        if (cmd_len > 0) {
            cmd_len--;
            HAL_UART_Transmit(&huart1, (uint8_t *)"\b \b", 3, 10);
        }
    } else if (cmd_len < CMD_BUF_SIZE - 1 && b >= 0x20 && b < 0x7F) {
        g_hist_pos = -1;
        cmd_buf[cmd_len++] = b;
        HAL_UART_Transmit(&huart1, &b, 1, 10);
    }
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* ===== UART 工具 ===== */
static void uart_print(const char *fmt, ...)
{
    static char buf[RESP_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}

static void show_prompt(void) { uart_print(PROMPT_STR); }

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
        g_leds[id].mode           = LED_MODE_BLINK;
        g_leds[id].blink_period_ms = period;
        g_leds[id].next_toggle    = HAL_GetTick() + period;
        uart_print("LED%d BLINK %d ms\r\n", id + 1, period);
    } else {
        uart_print("Error: unknown action '%s'\r\n", argv[1]);
    }
}

static void cmd_pwm(int argc, char *argv[])
{
    if (argc < 3) { uart_print("Usage: pwm <id> <duty 0-100>\r\n"); return; }
    int id = atoi(argv[1]), duty = atoi(argv[2]);
    if (id != 1) { uart_print("Error: only PWM 1 is supported\r\n"); return; }
    if (duty < 0 || duty > 100) { uart_print("Error: duty must be 0-100\r\n"); return; }
    uint32_t ccr = (htim1.Init.Period + 1) * duty / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    uart_print("PWM%d duty = %d%%\r\n", id, duty);
}

static void cmd_uptime(int argc, char *argv[])
{
    uint32_t ms = HAL_GetTick() - g_start_tick, s = ms / 1000;
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

static void isr_replace_line(const char *s, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\x1b[2K" PROMPT_STR, 5 + PROMPT_LEN, 20);
    if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)s, len, 50);
}

static void cmd_history(int argc, char *argv[])
{
    __disable_irq();
    uint8_t cnt = g_hist_count, head = g_hist_head;
    __enable_irq();
    if (cnt == 0) { uart_print("  (no history)\r\n"); return; }
    for (int8_t i = (int8_t)cnt - 1; i >= 0; i--) {
        uint8_t idx = (head - 1 - (uint8_t)i + HISTORY_SIZE * 2) % HISTORY_SIZE;
        uart_print("  %3d  %s\r\n", (int)(cnt - 1 - (uint8_t)i), g_history[idx]);
    }
}

/* ===== LED ===== */
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

/* ===== 初始化函数 ===== */

/**
 * C031C6 HSI 内置振荡器 = 48 MHz，直接作为 SYSCLK，无需 PLL。
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;   /* 48MHz ÷ 1 = 48MHz */
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA5 - LED1 LD2，初始低（灭） */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0, PB1 - LED2, LED3，初始低（灭） */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = UART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    /* C031C6 SYSCLK = 48MHz；48MHz / 48 = 1MHz 计数时钟，ARR=999 → 1kHz PWM */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 47;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim1);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim1);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_MspPostInit(&htim1);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
