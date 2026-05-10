# STM32 串口命令行控制器

> 一个运行在 STM32F103 上的命令行交互系统，通过串口实现类 Linux Shell 的命令交互，控制板上 LED、PWM 等外设。

## 项目简介

本项目实现了一个轻量级的嵌入式命令行系统。用户通过串口终端输入命令，STM32 解析后控制硬件。整个系统采用**可扩展的命令表 + 函数指针**架构，新增命令只需在命令表添加一行。

## 功能特性

- ✅ 类 Shell 交互体验：回显、退格、命令提示符
- ✅ 基于函数指针的命令分发机制（易扩展）
- ✅ 简洁高效的命令解析器（不依赖 strtok）
- ✅ 多 LED 独立控制（开关 / 闪烁，闪烁周期可调）
- ✅ PWM 占空比动态调节
- ✅ 系统运行时间、命令统计
- ✅ 完善的参数检查与错误提示
- ✅ 可在 Wokwi STM32 模拟器上验证

## 支持命令

| 命令 | 说明 | 示例 |
|------|------|------|
| `help` | 显示帮助 | `help` |
| `led on <id>` | 打开指定 LED | `led on 1` |
| `led off <id>` | 关闭指定 LED | `led off 2` |
| `led blink <id> <ms>` | LED 周期闪烁 | `led blink 1 500` |
| `led status` | 显示所有 LED 状态 | `led status` |
| `pwm <id> <duty>` | 设置 PWM 占空比 | `pwm 1 50` |
| `uptime` | 显示运行时间 | `uptime` |
| `stats` | 显示命令统计 | `stats` |
| `reset` | 软复位 | `reset` |

## 交互示例

```
=====================================
 STM32 Command Line Controller v1.0
 Type 'help' for command list
=====================================
stm32> help
Available commands:
  help     - Show all commands
  led      - led <on|off|blink|status> <id> [param]
  pwm      - pwm <id> <duty 0-100>
  uptime   - Show system uptime
  stats    - Show command statistics
  reset    - Soft reset

stm32> led blink 1 200
LED1 BLINK 200 ms

stm32> pwm 1 75
PWM1 duty = 75%

stm32> uptime
Uptime: 42.158 s  (0:00:42)

stm32> stats
System statistics:
  Commands processed: 5
  Command errors:     0
  Success rate:       100%
```

## 硬件要求

| 引脚 | 功能 |
|------|------|
| PC13 | LED1（板载，低电平点亮） |
| PB0  | LED2 |
| PB1  | LED3 |
| PA8  | TIM1_CH1 PWM 输出 |
| PA9  | USART1_TX |
| PA10 | USART1_RX |

也可以**直接在 Wokwi 上跑**，无需硬件：https://wokwi.com/projects/new/stm32

## 软件架构

### 命令表机制

```c
typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} Command_t;

static const Command_t cmd_table[] = {
    { "led", cmd_led, "..." },
    { "pwm", cmd_pwm, "..." },
    ...
};
```

新增命令只需：
1. 实现 `cmd_xxx(int argc, char *argv[])` 函数
2. 在 `cmd_table` 中加一行

不需要修改解析器，**符合开闭原则**。

### 命令处理流程

```
UART RX 中断
    ↓ (每字节)
缓冲到 cmd_buf，遇 \r\n 标记 cmd_ready
    ↓
主循环检测 cmd_ready
    ↓
分词 → 查命令表 → 调用 handler
    ↓
输出结果，重新显示提示符
```

### 为什么用查表 + 函数指针？

如果用 `if-else` 链分发命令，每加一条命令就要修改一处。命令表是**典型的"数据驱动"设计**，把"什么命令做什么"变成了静态数据，代码逻辑保持不变。这是工业代码常用的模式。

## 编译与运行

### 方式 1：Wokwi 在线模拟器（推荐先用这个验证）

1. 打开 https://wokwi.com/projects/new/stm32
2. 选择 STM32F103 板子
3. 添加 USART 连接，在右侧的串口监视器输入命令
4. 复制本项目的 `main.c`，按需调整引脚

### 方式 2：STM32CubeIDE + 真板子

1. 新建 STM32CubeIDE 项目，芯片 STM32F103C8Tx
2. 在 CubeMX 中配置：
   - PC13/PB0/PB1: GPIO Output
   - PA8: TIM1_CH1 PWM Generation
   - TIM1: Prescaler = 71, Period = 999 (1kHz PWM)
   - USART1: 115200 8N1, RX Interrupt 使能
3. 将 `src/main.c` 中的命令处理逻辑合并到生成的 `main.c`
4. 编译并烧录

### 测试

用串口终端工具（minicom / screen / PuTTY）连接：

```bash
# Mac
screen /dev/cu.usbserial-XXXX 115200

# Linux
sudo minicom -D /dev/ttyUSB0 -b 115200

# Windows
# 使用 PuTTY，COM 口 + 115200
```

## 项目结构

```
stm32-cli-controller/
├── src/
│   └── main.c           # 完整实现
├── docs/
│   └── architecture.md
└── README.md
```

## 代码统计

约 400 行 C 代码，包含完整命令解析、LED 状态机、PWM 控制、系统统计。

## 技术要点

1. **环形缓冲区思想**：UART 接收用单字节中断 + 缓冲区，主循环处理，**中断快进快出**
2. **状态机思想**：LED 闪烁不用阻塞 delay，主循环根据当前 tick 判断是否翻转
3. **可扩展架构**：命令表设计让新功能添加成本极低
4. **错误处理完整**：每个命令都做参数边界检查，统计错误率

## 学到了什么

- UART 中断接收的正确写法（不阻塞主循环）
- 函数指针 + 表驱动设计模式
- 简单命令行解析器的实现
- 用 `HAL_GetTick()` 实现非阻塞定时
- C 语言可变参数 (`vsnprintf`) 用法

## 许可证

MIT
