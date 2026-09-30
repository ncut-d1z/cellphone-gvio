# 04 IMU 与磁强计

> 硬件篇 · 惯性 / 磁力传感器
> 整理时间：2026-08

## 1. 配备情况

HONOR 50 Pro 配备以下运动与方位传感器：

| 传感器 | Android 类型 | 用途 |
|---|---|---|
| 加速度计 | ACCELEROMETER | 线加速度 + 重力方向 |
| 陀螺仪 | GYROSCOPE | 角速度 |
| 磁强计（罗盘） | MAGNETIC_FIELD | 地磁场三轴分量 |
| 距离传感器 | PROXIMITY | 非本项目用 |
| 环境光传感器 | LIGHT | 非本项目用 |

**无气压计**（不配备 PRESSURE 传感器）。

## 2. 芯片型号状态（重要）

- **公开渠道未公布**该机型 IMU / 磁强计的具体芯片型号（厂商拆机数据库为付费资料，如 inmobile.info）。
- 芯片厂商可能性：同代骁龙中端机常用 Bosch、STMicroelectronics、TDK InvenSense 等方案，但**不能凭猜测直接采用**其数据手册参数。
- 正确做法：在真机上通过 Android 运行时查询确认——
  - `Sensor.getName()`、`Sensor.getVendor()`、`Sensor.getVersion()`：识别芯片与驱动版本；
  - `Sensor.getMaximumRange()`、`Sensor.getResolution()`、`Sensor.getMinDelay()` / `getMaxDelay()`：量程、分辨率、支持的最高采样间隔；
  - 实测静止/转动时的噪声水平与漂移特性。
- 所有噪声密度、零偏稳定性等 IMU 误差参数**必须实测标定**，不得假设为某款芯片的数据。

## 3. 采样能力

- 典型 Android 手机 IMU 通过系统 HAL 暴露 100–400Hz 采样能力，骁龙平台常见 200–400Hz。
- 加速度计与陀螺仪通常集成于同一 IMU 芯片，二者时间戳同源，可近似视为同步。
- 磁强计采样率一般低于 IMU（常见 50–100Hz 档），且与 IMU 不保证同一时钟批次。
- 以上均为经验范围，实际以 `Sensor.getMinDelay()` 查询结果为准（**待实测**）。

## 4. 磁强计注意事项（硬件事实层面）

- 手机内磁强计为三轴磁力计，用于获取设备系下的地磁矢量。
- 需要**硬磁校准**（机内磁体/电流导致的零偏）与**软磁校准**（周边磁化材料导致的尺度/非正交误差）。
- 中国地区地磁偏角（磁北与真北夹角）随地域变化（约 -10° 至 +5° 量级，含 2026 年地磁场模型更新），将磁航向转为真北需查表/模型校正（如 WMM）。
- 手机内部扬声器、马达、充电电流、金属边框均会干扰磁力计读数；采集时避免靠近强磁物体，充电时读数会劣化。
- Android 系统提供厂商融合的 `TYPE_ROTATION_VECTOR` 结果，但本项目应读取**原始**磁强计数据（`TYPE_MAGNETIC_FIELD_UNCALIBRATED`）自行处理。

## 5. 对项目的数据接口事实

| 事实 | 说明 |
|---|---|
| 时间戳基准 | 传感器事件时间戳基于 `CLOCK_MONOTONIC`（纳秒），与相机、GNSS 时钟不同域，需对齐（见 [软件篇 05](../software/05-data-interface-notes.md)） |
| 坐标系 | 使用 Android 设备传感器坐标系（见 [软件篇 03](../software/03-sensor-api.md)） |
| 原始 vs 融合 | 系统可能叠加厂商自动校准；读取 UNCALIBRATED 变体并记录估计偏差量 |
| 温漂 | 连续高频采集发热，零偏随温度漂移，长时间采集建议分段标定 |
| 无气压计 | 高度方向无独立参考 |

## 参考来源

- GSMChina（传感器清单）：https://gsmchina.com/smartphones/honor-50-pro
- Gizmochina（传感器清单）：https://www.gizmochina.com/product/honor-50-pro
- inmobile.info 拆机库（付费，未获取芯片细节）：https://inmobile.info/boarden/13821/Honor_50_Pro_(HN2RNAM).html
- Android 传感器 API 文档：https://developer.android.com/develop/sensors-and-location/sensors
