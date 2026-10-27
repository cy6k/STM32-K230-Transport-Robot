# STM32F4 智能搬运小车

本工程是基于 STM32F4 的智能搬运小车项目，以完成 **2026 年四川省大学生工程实践与创新能力大赛搬运赛项**的任务为目标。

项目使用 STM32F407VET6，采用 Keil MDK 开发。后续将逐步完善小车运动控制及搬运任务相关功能，并在此记录开发进展。

## 工程结构

- `Hardware/`：硬件驱动代码
- `System/`：系统功能代码
- `User/`：应用程序代码
- `Libraries/`：STM32 相关库文件
- `Project.uvprojx`：Keil 工程文件

## 项目状态

开发中。

## 当前模块

- 底盘运动控制与运行时间估算（`Motor`）
- 机械臂和舵机控制（`Arm`、`Servo`）
- HWT101 陀螺仪数据采集与航向处理
- K230 视觉模块串口通信与定位处理（`VisionSerial`、`Vision_work`）
- 串口屏交互（`tjc`）
- 路线和任务流程（`Work`）
- 补光灯控制（`Light`）

使用 Keil MDK 打开 `Project.uvprojx` 进行编译。`Objects/`、`Listings/` 等编译输出和本机 uVision 配置由 `.gitignore` 排除。
