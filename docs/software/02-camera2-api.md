# 02 Camera2 API 能力

> 软件篇 · 相机采集接口
> 整理时间：2026-08

## 1. 概述

HONOR 50 Pro 基于 Android 11（API 30），第三方 App 通过 Camera2 API 访问后置主摄。所有能力需在真机用 `CameraCharacteristics` 运行时枚举确认，以下为该机型公开视频规格对应的预期能力（**待实测**）。

## 2. 分辨率 / 帧率（后置主摄，预期）

| 模式 | 预期能力 | 说明 |
|---|---|---|
| 4K（3840×2160） | 30fps | 官方视频规格 4K@30 |
| 1080p（1920×1080） | 30 / 60 fps | 官方确认；120/240 慢动作为专用模式 |
| 720p | 30–240 fps | 慢动作档 |

- 以上仅为预期，实际 `StreamConfigurationMap.getOutputSizes()` / `getHighSpeedVideoSizes()` 以真机为准。

## 3. 输出格式（对本项目相关）

| 格式 | 说明 |
|---|---|
| YUV_420_888 | 通用 YUV 输出，可直接喂给灰度特征提取，推荐采集格式 |
| PRIVATE（ImageReader 可选） | 厂商编码格式，不通用，避免使用 |
| RAW_SENSOR | 未处理 Bayer 原始数据；主摄（HM2 级别）支持 10/12-bit RAW，但**是否开放取决于厂商 HAL，待实测**；RAW 流与 YUV 流并行会增大带宽与功耗 |

## 4. 时间戳（对融合项目最关键的事实）

- 每一帧通过 `Image.getTimestamp()` / `CaptureResult.get(CaptureResult.SENSOR_TIMESTAMP)` 获得帧时间戳。
- 该时间戳属于**相机 SENSOR 时钟域**，与 IMU 的 `CLOCK_MONOTONIC` 是不同时钟源，二者之间存在**固定偏移 + 微小漂移**，采集时必须记录两路时间戳用于离线对齐（详见 [05-data-interface-notes.md](05-data-interface-notes.md)）。
- `SENSOR_INFO_TIMESTAMP_SOURCE` 表明系统是否保证 SENSOR 时钟与 `CLOCK_MONOTONIC` 对齐（`REALTIME` 或 `UNKNOWN`，需真机查询）。
- 曝光时间：`SENSOR_EXPOSURE_TIME` 字段，暗光下曝光变长，帧"中心时刻"约 = 帧时间戳 + 曝光时间/2，高动态场景需考虑。

## 5. 自动曝光 / 对焦 / 防抖（事实）

| 项目 | 说明 |
|---|---|
| AE/AF | 支持自动曝光与 PDAF 自动对焦，支持锁定（`CONTROL_AE_LOCK`、`CONTROL_AF_TRIGGER`） |
| 对焦模式 | 支持连续对焦与手动对焦（`LENS_FOCUS_DISTANCE`），固定焦距场景建议锁定对焦防止特征尺度突变 |
| 数字变焦 | 主摄支持 2–10x 数字变焦（裁剪传感器区域） |
| OIS | 无（硬件层面不存在） |
| EIS | 官方视频链路为 gyro-EIS；Camera2 第三方预览/采集流通常**不叠加** EIS 裁切，以真机行为为准（**待实测**） |

## 6. 多摄（事实）

- 后置四摄对应 4 个 camera ID（1 主摄 / 2 超广角 / 3 微距 / 4 景深），具体 ID 映射需 `CameraManager.getCameraIdList()` 运行时确认。
- 不同摄像头之间存在平移与视差，融合项目应固定使用主摄，不跨镜头。

## 7. 采集工程要点（事实层建议）

- 推荐采集配置：主摄 1080p@30 YUV_420_888，兼顾特征质量、带宽与算力。
- 1080p 原始 YUV 带宽 ≈ 1920×1080×1.5×30 ≈ 93 MB/s；4K 为该值的 4 倍，长时间采集存储压力大（详见 [05-data-interface-notes.md](05-data-interface-notes.md)）。
- 每帧记录：帧时间戳、曝光时间、ISO、对焦距离、AE 状态，供离线分析与时间对齐。

## 参考来源

- Android Camera2 文档：https://developer.android.com/media/camera/camera2
- GSMArena（视频规格）：https://www.gsmarena.com/honor_50_pro-10958.php
- Beebom（视频规格）：https://gadgets.beebom.com/mobile/honor-50-pro
