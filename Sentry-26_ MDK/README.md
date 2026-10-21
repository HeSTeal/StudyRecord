# Sentry-26 哨兵机器人

RoboMaster 哨兵机器人嵌入式工程，基于 STM32F405 平台。

- **当前版本**：v1.1
- **硬件平台**：STM32F405（工程名 `RM_F405_DEMO`）
- **框架**：湖南大学跃鹿战队（HNU YueLu EC）开源框架（"鹿框架"）
- **开发环境**：STM32CubeMX + Keil MDK-ARM + FreeRTOS

## 版本更新

### v1.1（当前）

- 主要更新：**导航模式（Navigation Mode）**
  - 新增导航协议解析与回传（`MODULES/master_machine/NAVI.h`、`master_process.c`）
  - 底盘新增 `CHASSIS_NAVIGATION` / `CHASSIS_COMPETITION` 模式
  - 云台新增 `Gimbal_NAVI` 控制模式（目标角 `GIMBAL_NAVI_YAW_REF`）
  - 基于 USB 虚拟串口（VCP）与导航系统通信

## 系统架构

工程采用 **双板架构**，通过宏定义区分板型：

- `GIMBAL_BOARD`（云台板）：云台姿态控制、发射、视觉/自瞄通信
- `CHASSIS_BOARD`（底盘板）：底盘运动、导航、裁判系统通信

> 当前 `APP/robot_def.h` 中使能 `GIMBAL_BOARD`，烧录时注意只能保留一个板型定义。

### 目录结构

- `APP/`：应用层（`chassis` 底盘 / `gimbal` 云台 / `shoot` 发射 / `cmd` 命令 / `robot` 框架入口）
- `BSP/`：板级支持包（CAN / SPI / USART / USB / PWM / DWT / GPIO / RS485）
- `MODULES/`：功能模块
  - `algorithm/`：PID 控制器、卡尔曼滤波、四元数 EKF（QEKF）、CRC、通信协议
  - `imu/`：BMI088 驱动与 INS 姿态解算
  - `motor/`：DJI 电机 / 达妙（DM）电机驱动与电机任务
  - `master_machine/`：上位机通信（导航 `NAVI` / 自瞄 `Auto_Aim` / 视觉类型定义）
  - `message_center/`：发布订阅消息中心
  - `remote/`：遥控器解析
  - `can_comm/`、`daemon/` 等辅助模块
- `Core/`：CubeMX 生成的 HAL 外设初始化
- `Drivers/`：HAL 库与 CMSIS
- `Middlewares/`：FreeRTOS / USB Device
- `USB_DEVICE/`：USB 虚拟串口（CDC）
- `MDK-ARM/`：Keil 工程

## 云台反馈源

云台 YAW 轴支持两种反馈源（`APP/gimbal/gimbal.c`）：

- **编码器反馈**：电机 `total_angle` / `velocity`
- **陀螺仪（IMU）反馈**：IMU `YawTotalAngle` / `Gyro`

通过 `Feedback_Source_e`（`OTHER_FEED` / `MOTOR_FEED`）与 `YawSetFeedSource()` 切换，
两套 PID 参数相互独立（`yaw_*_cfg_imu` / `yaw_*_cfg_enc`），切换时重新初始化 PID 以清空历史状态。

## TODO

- [ ] **自瞄（Auto-Aim）**：接入 miniPC 视觉数据，完成云台自动瞄准闭环
- [ ] **反馈源切换**：支持在编码器反馈与陀螺仪反馈之间（及对应控制策略）稳定切换

## 构建

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/RM_F405_DEMO.uvprojx`
2. 在 `APP/robot_def.h` 中选择目标板型（`GIMBAL_BOARD` / `CHASSIS_BOARD`）
3. 编译并烧录

> 修改外设配置可先使用 STM32CubeMX 打开 `RM_F405_DEMO.ioc`。
