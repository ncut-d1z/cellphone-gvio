# 02 Snapdragon 778G 5G（SM7325）SoC

> 硬件篇 · 处理器平台
> 整理时间：2026-08

## 1. 概述

HONOR 50 Pro 搭载 Qualcomm Snapdragon 778G 5G 移动平台（Part Number：SM7325），是 2021 年发布的中高端 SoC，采用台积电 6nm 制程。本项目的数据采集与（手机本地运行的）融合计算均在该平台上进行。

| 项目 | 参数 |
|---|---|
| 型号 | Snapdragon 778G 5G（SM7325） |
| 制程 | 6nm（TSMC） |
| 发布时间 | 2021-05 |
| CPU | Kryo 670：1×2.4GHz Cortex-A78 + 3×2.2GHz Cortex-A78 + 4×1.9GHz Cortex-A55 |
| GPU | Adreno 642L（OpenGL ES 3.2 / Vulkan 1.1 / OpenCL 2.0 FP） |
| DSP/AI | Hexagon 770（Tensor + Vector + Scalar 加速器），第 6 代 AI Engine，最高 12 TOPS |
| 内存支持 | LPDDR5 @3200MHz 或 LPDDR4x @2133MHz，最高 16GB |
| 调制解调器 | Snapdragon X53 5G（mmWave + Sub-6，NSA/SA，下行最高约 3.3Gbps） |
| 连接 | FastConnect 6700：Wi-Fi 6E 硬件、蓝牙 5.2、aptX 系列、Snapdragon Sound |

## 2. CPU（Kryo 670）

- 架构：1 大核 + 3 中核（Cortex-A78）+ 4 小核（Cortex-A55）的 big.LITTLE 结构。
- 大核 2.4GHz 单核性能最强，适合耗时任务（视觉特征提取、矩阵运算）。
- 小核 1.9GHz 适合后台传感器数据搬运。
- 对项目：融合计算建议绑定大核（Android 可通过线程优先级/affinity 优化），但手机存在温控降频，需在预算中留余量。

## 3. GPU（Adreno 642L）

- 支持 OpenGL ES 3.2、Vulkan 1.1、OpenCL 2.0 FP。
- 对项目：可考虑将图像预处理（灰度化、降采样）卸载到 GPU/OpenCL，但会增加功耗与发热。

## 4. DSP / AI（Hexagon 770）

- 6th Gen Qualcomm AI Engine：Hexagon Tensor Accelerator + Vector eXtensions + Scalar Accelerator，峰值约 12 TOPS。
- 对项目：Qualcomm 神经处理 SDK（QNN）可将特征提取等算子部署到 DSP，降低 CPU 占用；但 778G 为中端平台，实际可用算力需基准测试验证。

## 5. 2nd Gen Sensing Hub（第二代传感中枢）

- 常开、低功耗（宣传 <1mA）的专用低功耗处理器。
- 职责：音频、传感器、Wi-Fi、位置、BLE 等上下文数据流的常开后台处理与融合。
- 对项目：系统级姿态/步数等融合结果由 Sensing Hub 提供（SensorManager 的 TYPE_ROTATION_VECTOR 等可能来自该通路）；本项目需要的是**原始**传感器数据（UNCALIBRATED 变体），应绕过厂商融合结果直接读取硬件传感器。

## 6. ISP：Qualcomm Spectra 570L

| 能力 | 参数 |
|---|---|
| 类型 | 三路 14-bit ISP |
| 吞吐 | 最高 2 Gigapixels/s |
| 单摄 ZSL | 最高 64MP @30fps（零快门延迟） |
| 双摄 ZSL | 最高 36MP + 22MP @30fps |
| 三摄 ZSL | 最高 22MP @30fps |
| 拍照 | 最高 200MP 照片拍摄 |
| 视频 | 4K@30 HDR 视频、HEVC/HEIF、10-bit 色深、HDR10+/HDR10/HLG、多帧降噪、Staggered HDR 传感器支持 |

- 对项目：ISP 支持 4K@30 硬件管线；第三方 App 通过 Camera2 API 可访问 YUV/RAW 输出，ISP 的高吞吐保证 1080p/4K 30fps 稳定采集。

## 7. GNSS 硬件能力（SoC 层面）

| 能力 | 参数 |
|---|---|
| 星座 | GPS、GLONASS、BeiDou、Galileo、QZSS、NavIC |
| 频率 | 双频 L1/L5（Dual Frequency Support） |
| 特色 | Sidewalk/Lane-level Positioning（人行道/车道级定位）、Sensor-Assisted Positioning（传感器辅助定位） |

> 手机整机实际开放的星座与频段见 [05-gnss-hardware.md](05-gnss-hardware.md)，需实测验证。

## 8. 对项目的性能预算参考

- 8GB 内存：应用进程可用内存由系统分配（Android 默认 heap 上限约 256–512MB，Native 内存另计），实际可用需压测。
- 两个 Cortex-A78 大核可承担主要计算；持续高负载会触发温控降频（6nm 中端机典型热设计），实时处理需预留 30–50% 余量。
- 全速采集（相机 30fps + IMU 200Hz + GNSS 1Hz）时，建议融合计算在独立线程/核上运行。

## 参考来源

- Qualcomm Snapdragon 778G 5G 产品简报（PDF）：https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/Snapdragon-778G-5G-Product-Brief.pdf
- XDA Developers（778G 发布分析）：https://www.xda-developers.com/qualcomm-snapdragon-778g/
- NotebookCheck（SD 778G 页面）：https://www.notebookcheck.net/Qualcomm-Snapdragon-778G-5G-Processor-Benchmarks-and-Specs.560437.0.html
- PhoneDB：https://phonedb.net/index.php?m=processor&id=932&d=detailed_specs
