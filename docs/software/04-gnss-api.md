# 04 定位 API（LocationManager / GnssMeasurement）

> 软件篇 · GNSS 采集接口
> 整理时间：2026-08

## 1. 接口总览（Android 11 / API 30）

| 接口 | 用途 | 本项目 |
|---|---|---|
| LocationManager / FusedLocationProvider | 融合位置（GPS + 网络 + 传感器） | 位置量测来源 |
| GPS_PROVIDER | 纯 GNSS 位置 | 推荐，避免网络定位污染 |
| GnssMeasurement API | 卫星原始观测量（伪距/载波相位/多普勒） | 高级量测来源，需厂商开放 |
| GnssStatus API | 卫星可见性与信号强度 | 采集状态监控 |

## 2. LocationManager 事实

| 项 | 说明 |
|---|---|
| 更新率 | 消费级手机典型 1Hz（可请求 minTime 提高，实际受 HAL 限制） |
| 数据 | 经纬度（WGS84）、海拔、地面速度（m/s）、方位、水平精度（米）、垂直精度（部分设备） |
| 时间戳 | `Location.getTime()` 基于 **UTC（elapsedRealtime 为单调钟）**；与 IMU/相机时钟不同域 |
| 精度 | 单频 GNSS 典型 5–10m；城市多路径下更差 |
| 权限 | `ACCESS_FINE_LOCATION` + 系统定位开关 |

## 3. GnssMeasurement（原始观测量）事实

| 项 | 说明 |
|---|---|
| 引入版本 | Android 7（API 24），API 30 下完整可用 |
| 内容 | 每颗卫星：伪距（原始计数/米）、载波相位、多普勒、载噪比（Cn0）、星座类型、载波频率 |
| 频率字段 | `CarrierFrequencyHz`：L1=1575.42MHz、L5=1176.45MHz；**双频是否开放以此字段实测确认**（见 [硬件篇 05](../hardware/05-gnss-hardware.md)） |
| 时间 | `GnssClock`：GPS 时系（周 + 毫秒 TOW），并有与手机 `CLOCK_MONOTONIC` 的关联字段（如 `ReceiverTimeOffsetNS`） |
| 更新 | 典型按 1Hz 批次回调（真机为准，**待实测**） |
| 可用性 | 取决于厂商 HAL 实现；需室外开阔天空，室内/隧道无数据 |
| 状态 | `GnssStatus` 提供星座/载噪比/用于历书状态，可监控定位质量 |

> 注意：部分机型即使硬件支持双频，HAL 也可能不向第三方 App 暴露 L5 原始观测量。真机上直接检查回调中的 `CarrierFrequencyHz` 与 `ConstellationType` 分布即可判定。

## 4. 时间基准关系（事实）

| 时钟域 | 基准 | 载体 |
|---|---|---|
| 传感器（IMU/磁强计） | CLOCK_MONOTONIC（纳秒） | SensorEvent.timestamp |
| 相机 | 相机 SENSOR 时钟域（`SENSOR_INFO_TIMESTAMP_SOURCE` 可能为 REALTIME/UNKNOWN） | Image.getTimestamp() |
| GNSS | GPS 时系（TOW，需经 GPS 周/闰秒换算为 UTC） | GnssClock |
| 位置 | UTC + elapsedRealtime | Location |

三域之间的偏移需在采集阶段记录足够信息（如 `GnssClock` 中与单调钟的关联字段、相机 `SENSOR_INFO_TIMESTAMP_SOURCE`）用于离线对齐，详见 [05-data-interface-notes.md](05-data-interface-notes.md)。

## 5. 采集工程要点（事实层建议）

- 使用 `LocationManager.requestLocationUpdates(GPS_PROVIDER, ...)` 获取纯 GNSS 位置，附带 `mSpeed`、`mAccuracy`。
- 并行注册 `GnssMeasurements.Callback`，记录每次回调的 `GnssClock` 与全部卫星测量。
- 记录定位状态：可见卫星数、星座分布、载噪比，用于评估每次量测的可用性。
- 冷启动等待时间可能达数十秒，采集开始前先预热定位。

## 参考来源

- Android GNSS 定位文档：https://developer.android.com/develop/sensors-and-location/sensors/gnss
- Android Location 文档：https://developer.android.com/develop/sensors-and-location/location/update-location
- Qualcomm Snapdragon 778G 产品简报（GNSS 能力）：https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/Snapdragon-778G-5G-Product-Brief.pdf
