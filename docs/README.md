# HONOR 50 Pro 软硬件知识库

本项目（单目视觉 + IMU + 磁强计 + GPS 的 MSCKF 融合系统）数据源为 HONOR 50 Pro 手机。本目录为该型手机的软硬件技术知识库，全部内容仅涉及手机本身的能力与参数，不含融合算法原理。

## 文档索引

### 硬件篇 `hardware/`

| 文档 | 内容 |
|---|---|
| [01-device-overview.md](hardware/01-device-overview.md) | 整机规格：型号、尺寸、屏幕、电池、存储、网络、连接、传感器清单 |
| [02-soc-snapdragon-778g.md](hardware/02-soc-snapdragon-778g.md) | Snapdragon 778G (SM7325)：CPU/GPU/DSP/AI、Spectra 570L ISP、Sensing Hub、GNSS 硬件能力 |
| [03-camera-hardware.md](hardware/03-camera-hardware.md) | 相机硬件：后置四摄、前置双摄、视频规格、无 OIS |
| [04-imu-magnetometer.md](hardware/04-imu-magnetometer.md) | IMU 与磁强计：芯片型号状态、采样率、校准、无气压计 |
| [05-gnss-hardware.md](hardware/05-gnss-hardware.md) | GNSS 硬件：星座支持、L1/L5 双频能力、精度预期 |

### 软件篇 `software/`

| 文档 | 内容 |
|---|---|
| [01-os-platform.md](software/01-os-platform.md) | 操作系统：Android 11 / Magic UI 4.2、API 30、权限与生命周期 |
| [02-camera2-api.md](software/02-camera2-api.md) | Camera2 API：可用分辨率/帧率、帧时间戳时钟域、RAW、EIS |
| [03-sensor-api.md](software/03-sensor-api.md) | SensorManager：传感器类型、UNCALIBRATED 变体、采样率、时间戳与坐标系 |
| [04-gnss-api.md](software/04-gnss-api.md) | 定位 API：LocationManager、GnssMeasurement 原始观测量、时间基准 |
| [05-data-interface-notes.md](software/05-data-interface-notes.md) | 数据接口事实：各源采样率、时钟域、坐标系、存储/算力/热量预算 |

## 设备标识

- 机型：HONOR 50 Pro
- 型号代码：HN2RNAM
- 国行首发价：¥3699（8GB+256GB）
- 发布日期：2021-06-16 中国发布 / 2021-06-25 全球发售

## 重要说明

1. 本文档整理于 2026-08，信息来自公开渠道（GSMArena、Qualcomm 官方产品简报、多家规格站、Android 官方文档等），详见各篇末尾"参考来源"。
2. 部分参数在公开资料中存在分歧或未公开，文档中以"待实测"标注，需在真机上通过运行时查询验证：
   - IMU/磁强计具体芯片型号（厂商拆机库为付费资料）
   - GNSS 双频 L1/L5 原始观测量是否实际开放
   - Camera2 RAW 输出可用性
   - 各传感器实际最高采样率
3. 该机型软件支持期已于 2026 年结束（出厂 Android 11），本项目需基于出厂系统能力进行采集。
