# TTC_stm32f4

STM32F407 步进电机控制工程，基于野火骄阳开发板例程整理而来，当前版本主要用于双路步进电机的独立控制与串口调试。

## 项目简介

当前工程实现了两台步进电机的非阻塞控制：

- `mtor1`
- `mtor2`

主要特性：

- 基于 `TIM8` 输出比较 Toggle 模式产生步进脉冲
- 双通道独立控制
- 串口名称式命令控制
- 运行一台电机时不阻塞另一台电机接收命令
- 两台电机当前统一配置为：
  - `200步/圈`
  - `8细分`
  - `1:1直驱`

## 当前命令

```text
mtor1 turn [rev]
mtor1 move [rev] [accel] [decel] [rpm]
mtor1 accel [value]
mtor1 decel [value]
mtor1 rpm [value]
mtor1 status

mtor2 turn [rev]
mtor2 move [rev] [accel] [decel] [rpm]
mtor2 accel [value]
mtor2 decel [value]
mtor2 rpm [value]
mtor2 status

?
status
```

当前输入单位：

- `rev`：`0.1圈`
- `accel`：`rpm/s`
- `decel`：`rpm/s`
- `rpm`：`rpm`

## 主要目录

- `User/`：用户代码
- `Libraries/`：CMSIS 与 STM32 HAL 库
- `Project/`：Keil 工程文件
- `Doc/`：项目说明文档

## 编译说明

开发环境：

- Keil MDK
- ARM Compiler 5

如果源码已经统一为 UTF-8，而编译时出现下面这类报错：

```text
invalid multibyte character sequence
missing closing quote
expected a ")"
```

可以在 Keil 中增加如下编译选项：

1. 选择 `Options for Target...`
2. 进入 `C/C++` 标签页
3. 在 `Misc Controls` 输入框中添加：

```text
--no-multibyte-chars
```

该选项可避免编译器按多字节字符方式处理源码中的中文字符串，从而绕开 UTF-8 中文源码导致的相关报错。

## 说明文档

当前代码对应的修改说明见：

- `Doc/双电机步进控制修改说明.md`
