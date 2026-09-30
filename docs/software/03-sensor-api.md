# 03 SensorManager（IMU / 磁强计采集接口）

> 软件篇 · 传感器 API
> 整理时间：2026-08

## 1. 相关传感器类型

HONOR 50 Pro（Android 11 / API 30）通过 `SensorManager` 暴露以下与本项目相关的传感器（真机 `SensorManager.getSensorList` 为准，**待实测**）：

| 类型 | 说明 | 数据 |
|---|---|---|
| TYPE_ACCELEROMETER | 加速度计（含重力） | 3 轴 m/s² |
| TYPE_ACCELEROMETER_UNCALIBRATED | 加速度计原始值 + 估计偏差 | 6 值（[x,y,z] 与 [bias_x,bias_y,bias_z]） |
| TYPE_GYROSCOPE | 陀螺仪 | 3 轴 rad/s |
| TYPE_GYROSCOPE_UNCALIBRATED | 陀螺仪原始值 + 估计漂移偏差 | 6 值 |
| TYPE_MAGNETIC_FIELD | 磁强计 | 3 轴 µT |
| TYPE_MAGNETIC_FIELD_UNCALIBRATED | 磁强计原始值 + 硬磁偏置估计 | 6 值 |
| TYPE_ROTATION_VECTOR | 厂商/系统融合姿态（**非原始**，供参考对照） | 四元数/欧拉 |

> 本融合项目应采集 **UNCALIBRATED** 变体：保留原始读数与系统估计偏差，便于在融合侧自行处理，避免叠加厂商自动校准（厂商校准在运动过程中可能突变）。

## 2. 采样率（事实与查询方法）

| 档位 | 说明 |
|---|---|
| SENSOR_DELAY_FASTEST | 传感器最高支持速率 |
| SENSOR_DELAY_GAME | 典型 50Hz 级 |
| SENSOR_DELAY_UI / NORMAL | 更低 |

- 真机实际最高速率用 `Sensor.getMinDelay()`（微秒）查询；同代骁龙中端机 IMU 通常支持 200–400Hz，磁强计通常 50–100Hz（**待实测**）。
- 采样事件按传感器 FIFO 批量上报，事件之间的实际时间间隔以 `SensorEvent.timestamp` 为准，不能按名义速率假设等间隔。

## 3. 时间戳（关键事实）

- `SensorEvent.timestamp`：纳秒，基准为 **`CLOCK_MONOTONIC`**（系统启动以来，不受系统时间调整影响）。
- 加速度计与陀螺仪若为同一 IMU 芯片，时间戳同源近似同步；磁强计为独立芯片，时间戳可不同源。
- 与相机帧时间戳（SENSOR 时钟域）**不同域**；与 GNSS 时间（GPS 时系）**不同域**。采集时三路时间戳需各自完整记录（见 [05-data-interface-notes.md](05-data-interface-notes.md)）。

## 4. 坐标系（事实定义）

Android 传感器坐标系（自然方向竖屏，设备正面朝向使用者）：

| 轴 | 方向 |
|---|---|
| X | 水平向右（设备右边缘方向） |
| Y | 设备顶端方向（竖屏朝上） |
| Z | 垂直屏幕向外（面向使用者） |

- 加速度计：+Z 与重力方向相反（平放时读 +9.81 m/s²）。
- 磁强计：地磁矢量在设备系下的三轴分量。
- 相机坐标系为（右、下、前），与传感器系之间存在固定旋转（外参，需标定）。

## 5. 校准与干扰（事实）

| 项 | 说明 |
|---|---|
| 陀螺仪零偏 | 静态漂移随时间/温度变化，可用 UNCALIBRATED 的 bias 字段观察 |
| 加速度计 | 需六面静态标定（重力翻转法）确定尺度/非正交，**真机实测** |
| 磁强计 | 需硬磁/软磁标定；手机内部磁体（扬声器/马达）、充电、金属环境均会干扰 |
| 温漂 | 高频采样发热，长时间采集零偏会漂移 |
| 后台限制 | Android 11 无传感器后台节流，但 Doze/厂商杀进程仍可能中断采集 |

## 6. 采集工程要点（事实层建议）

- 同时注册 ACCEL + GYRO + MAG 的 UNCALIBRATED 变体，使用 `SENSOR_DELAY_FASTEST`，事件回调线程设为高优先级。
- 每批事件记录 `timestamp`（纳秒）；不依赖回调顺序（不同传感器事件可能交错）。
- 采集期间记录电池充电状态（充电会恶化磁强计读数）。

## 参考来源

- Android 传感器概述：https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview
- Android 运动传感器（时间戳/坐标系）：https://developer.android.com/develop/sensors-and-location/sensors/sensors_motion
- Android 位置传感器（磁强计/校准）：https://developer.android.com/develop/sensors-and-location/sensors/sensors_position
- GSMChina（传感器清单）：https://gsmchina.com/smartphones/honor-50-pro
