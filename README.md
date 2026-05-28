# STM32 串口命令行控制器

> 运行在 STM32C031C6（Cortex-M0+，48 MHz）上的轻量级嵌入式 CLI 系统。通过串口实现类 Shell 的人机交互，兼具工程实用性与代码可读性：命令表驱动、非阻塞状态机、VT100 终端协议解析，以及运行时自检诊断。

## 功能特性

| 技术点 | 实现 |
|--------|------|
| 命令分发 | 函数指针表（Open/Closed 原则）—— 新增命令只加一行 |
| 非阻塞 LED 闪烁 | 状态机 + `millis()` 时基，零 `delay()`，不影响串口 |
| 非阻塞 PWM 渐变 | 呼吸灯状态机，`pwm fade <step> <ms>` 启动 / `pwm stop` 停止 |
| VT100 终端协议 | ESC `[A`/`[B` 识别方向键，翻阅 8 条命令历史 |
| 命令历史 | 环形缓冲区，`↑↓` 翻历史，`Ctrl+C` 清行 |
| 系统诊断 | `sysinfo` 实时输出 MCU 型号 / 时钟 / 引脚图 / PWM 状态 |
| 统计监控 | `stats` 显示命令总数、错误率，便于测试阶段观测稳定性 |
| 参数校验 | 每条命令做完整边界检查，返回明确错误提示 |

## 命令参考

```
stm32> help
Available commands:
  help     - Show all commands
  led      - led <on|off|blink|status> <id> [ms]
  pwm      - pwm <id> <duty 0-100> | pwm fade <step> <ms> | pwm stop
  uptime   - Show system uptime
  sysinfo  - Show MCU info and pin map
  stats    - Show command statistics
  history  - Show command history (up/down arrows)
  reset    - Soft reset
```

### 命令示例

```bash
# LED 控制
led on 1                 # 点亮 LED1（PA5 板载 LD2）
led blink 2 300          # LED2 以 300 ms 周期闪烁（非阻塞）
led status               # 查看所有 LED 当前模式

# PWM 控制
pwm 1 75                 # 设置 PA8 占空比 75%（固定）
pwm fade 5 20            # 启动呼吸灯：每 20 ms 步进 5%（0→100→0 循环）
pwm stop                 # 停止渐变，PA8 输出归零

# 系统信息
sysinfo                  # 显示 MCU 型号、时钟、引脚图、PWM 实时状态
uptime                   # 运行时间（精确到毫秒）
stats                    # 命令处理数 / 错误率
```

## 交互演示

```
=====================================
 STM32 Command Line Controller v1.1
 Type 'help' for command list
=====================================
stm32> sysinfo
=== System Information ===
  MCU       : STM32C031C6  (Cortex-M0+)
  Clock     : 48 MHz  (HSI, no PLL)
  Flash     : 32 KB    RAM: 12 KB
  Framework : Arduino STM32 (SrcWrapper)
  Firmware  : CLI-Controller v1.1
  Build     : May 28 2026 10:34:17

=== Pin Map ===
  PA2  TX   -> Serial Monitor (115200 baud)
  PA3  RX   -> Serial Monitor
  PA5  OUT  -> LED1 (board LD2, active-high)
  PA8  PWM  -> LED4 yellow  [pwm 1 <duty>]
  PB0  OUT  -> LED2 green
  PB1  OUT  -> LED3 blue

=== Runtime ===
  Uptime    : 0:02:14
  Tick      : 134021 ms
  PWM fade  : RUNNING  duty=63%  step=5%  interval=20 ms

stm32> led blink 1 200
LED1 BLINK 200 ms

stm32> pwm fade 5 20
PWM fade ON: step=5% every 20 ms  (0->100->0 loop)

stm32> stats
System statistics:
  Commands processed: 8
  Command errors:     0
  Success rate:       100%
```

> **提示**：串口终端支持 `↑↓` 方向键翻阅历史命令、`Backspace` 退格、`Ctrl+C` 清行——VT100 ESC 序列由固件解析，体验接近真实 shell。

## 技术要点

### 1. 非阻塞呼吸灯状态机

许多教程用 `delay()` 实现渐变——这会阻塞整个 MCU，串口在渐变期间完全失去响应。  
本项目使用状态机 + 时间戳，`loop()` 每帧仅判断"是否到达步进时刻"，不做任何等待：

```c
typedef struct {
    uint8_t  active;       /* 1 = 正在渐变        */
    int8_t   direction;    /* +1=加亮  -1=减暗    */
    uint8_t  step;         /* 每步 duty 变化量    */
    uint32_t interval_ms;  /* 步进间隔 ms         */
    uint32_t next_ms;      /* 下次步进的绝对时刻  */
    int16_t  current;      /* 当前 duty (0-100)   */
} PwmFade_t;

static void update_pwm_fade(void)
{
    if (!g_fade.active) return;
    uint32_t now = millis();
    if (now < g_fade.next_ms) return;          // 未到时刻，立即退出

    g_fade.next_ms = now + g_fade.interval_ms;
    g_fade.current += g_fade.direction * (int16_t)g_fade.step;

    if      (g_fade.current >= 100) { g_fade.current = 100; g_fade.direction = -1; }
    else if (g_fade.current <= 0)   { g_fade.current = 0;   g_fade.direction = +1; }

    analogWrite(PA8, (int)(g_fade.current * 255 / 100));
}
```

同样的思路适用于所有"定时触发但不能阻塞"的场景（传感器采样、喂狗、心跳帧）。

### 2. VT100 终端协议解析

串口按字节到达，方向键实际上是三字节 ESC 序列：`0x1B 0x5B 0x41`（↑）。  
用一个两位 ESC 状态机识别，无额外内存开销：

```c
if (g_esc_state == 1) {
    g_esc_state = (b == '[') ? 2 : 0;  // 等待 '['
    return;
}
if (g_esc_state == 2) {
    g_esc_state = 0;
    if      (b == 'A') /* ↑ 上翻历史 */ ;
    else if (b == 'B') /* ↓ 下翻历史 */ ;
    return;
}
if (b == 0x1B) { g_esc_state = 1; }
```

### 3. 命令表 + 函数指针（Open/Closed 原则）

```c
typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} Command_t;

static const Command_t cmd_table[] = {
    { "help",    cmd_help,        "Show all commands"           },
    { "led",     cmd_led,         "led <on|off|blink|status> …" },
    { "pwm",     cmd_pwm,         "pwm <id> <duty> | fade | stop" },
    { "uptime",  cmd_uptime,      "Show system uptime"          },
    { "sysinfo", cmd_sysinfo,     "Show MCU info and pin map"   },
    { "stats",   cmd_stats,       "Show command statistics"     },
    { "history", cmd_history_cmd, "Show command history"        },
    { "reset",   cmd_reset,       "Soft reset"                  },
    { NULL, NULL, NULL }
};
```

新增命令 = 实现一个 `cmd_xxx()` + 在表里加一行。解析器代码零修改，符合 **OCP**。

### 4. 命令历史环形缓冲区

```
g_history[8][64]   ← 最近 8 条命令的环形缓冲
g_hist_head        ← 写指针（模 8 循环）
g_hist_pos         ← 当前翻阅偏移（-1 = 未翻阅）
```

按 ↑ 时 `offset++`，按 ↓ 时 `offset--`，`history_get(offset)` 通过偏移量计算真实数组下标，无需搬移数据。

## Wokwi 在线仿真（无需硬件，0 分钟上手）

**1. 打开 Wokwi 新建项目**

浏览器访问 https://wokwi.com → **New Project** → **STM32 Nucleo-64 C031C6**

**2. 替换两个文件**

| 文件 | 内容来源 |
|------|---------|
| `main.c` | 本仓库 [`wokwi/main.c`](wokwi/main.c) |
| `diagram.json` | 本仓库 [`wokwi/diagram.json`](wokwi/diagram.json) |

**3. 启动仿真**

点击 **▶** 按钮，等待编译（约 5–10 秒），右侧 **Serial Monitor** 出现提示符后即可输入命令。

### 推荐测试序列

```bash
sysinfo                  # 查看 MCU 信息
led blink 1 200          # LED1 以 200 ms 闪烁
led blink 2 700          # LED2 以 700 ms 闪烁（两灯异步，互不干扰）
pwm fade 5 20            # 启动呼吸灯（此时 LED 闪烁仍正常——验证非阻塞）
uptime                   # 查看运行时间
stats                    # 查看命令统计
pwm stop                 # 停止呼吸灯
led off 1                # 关闭 LED1
history                  # 查看历史记录
```

### 电路连接（diagram.json 已配置）

| 组件 | 引脚 | 命令 |
|------|------|------|
| 板载 LD2 | PA5（高有效） | `led on/off/blink 1` |
| 绿色 LED | PB0 → 220Ω → GND | `led on/off/blink 2` |
| 蓝色 LED | PB1 → 220Ω → GND | `led on/off/blink 3` |
| 黄色 LED | PA8 → 220Ω → GND | `pwm 1 <0-100>` / `pwm fade` |

## 项目结构

```
stm32-cli-controller/
├── wokwi/
│   ├── main.c           # 完整独立版（Arduino STM32 框架，可直接 Wokwi 运行）
│   └── diagram.json     # Wokwi 电路定义
├── src/
│   └── main.c           # 裸 HAL 版本（STM32F103，配合 CubeIDE 使用）
└── README.md
```

## 在真实硬件上运行（STM32F103 Blue Pill）

1. STM32CubeIDE 新建项目，芯片 **STM32F103C8Tx**
2. CubeMX 配置：
   - `PC13 / PB0 / PB1`：GPIO Output
   - `PA8`：TIM1_CH1 PWM Generation，PSC=71，Period=999（1 kHz）
   - `USART1`：115200 8N1，使能 RX 中断
3. 将 `src/main.c` 的命令逻辑整合进 CubeIDE 工程（或直接修改 `wokwi/main.c` 替换 Arduino API 为 HAL 调用）
4. 烧录后用终端工具连接：

```bash
# macOS
screen /dev/cu.usbserial-XXXX 115200

# Linux
minicom -D /dev/ttyUSB0 -b 115200

# Windows：PuTTY → Serial → COMx → 115200
```

## 许可证

MIT
