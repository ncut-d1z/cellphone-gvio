# 05 数据接口事实汇总（采样率 / 时钟 / 坐标 / 资源预算）

> 软件篇 · 对本项目数据接口的客观约束（仅手机相关事实，不涉及融合算法）
> 整理时间：2026-08

## 1. 数据源总览

| 数据源 | 接口 | 典型采样率 | 时钟域 | 坐标系 |
|---|---|---|---|---|
| 加速度计 | SensorManager（UNCALIBRATED） | 200–400Hz（待实测） | CLOCK_MONOTONIC | 设备传感器系 |
| 陀螺仪 | SensorManager（UNCALIBRATED） | 200–400Hz（待实测） | CLOCK_MONOTONIC | 设备传感器系 |
| 磁强计 | SensorManager（UNCALIBRATED） | 50–100Hz（待实测） | CLOCK_MONOTONIC | 设备传感器系 |
| 相机主摄 | Camera2（YUV_420_888） | 30fps（1080p/4K） | 相机 SENSOR 时钟域 | 相机系（右/下/前） |
| GNSS 原始观测 | GnssMeasurement | ~1Hz（待实测） | GPS 时系 | 卫星/ECEF |
| GNSS 位置 | LocationManager（GPS_PROVIDER） | ~1Hz | UTC | WGS84 地理系 |

## 2. 时钟域差异（客观事实）

- 三类时钟彼此独立，采集时必须三路**同时记录原始时间戳**，不得互相推算：
  - 传感器：`CLOCK_MONOTONIC`（单调，不受 NTP/用户改时间影响）；
  - 相机：SENSOR 时钟域，与单调钟偏移未知（`SENSOR_INFO_TIMESTAMP_SOURCE` 指示是否 REALTIME）；
  - GNSS：GPS 时系（GnssClock 提供与单调钟关联字段），位置为 UTC。
- 对齐可行性依赖：相机与单调钟的偏移（离线估计）、GNSS 与单调钟的关联字段。这些信息在采集日志中必须保留。

## 3. 坐标系（客观定义）

| 系 | 定义 |
|---|---|
| 设备传感器系 | Android 标准：X 右、Y 上（竖屏顶端）、Z 垂直于屏幕向外 |
| 相机系 | 右、下、前（Camera2 惯例） |
| 磁强计 | 设备传感器系下的地磁三轴分量 |
| GNSS 位置 | WGS84 经纬度/海拔（可换算 ECEF） |
| 姿态参考 | 地面航向通常以真北为参考，磁航向需磁偏角校正（中国地区约 -10°～+5°，随地区/年份变化） |

- 相机系与设备传感器系之间存在固定的旋转/平移（外参），出厂未知，**需标定**（工程事项，非算法原理）。

## 4. 数据量与存储预算（客观计算）

| 项 | 计算 | 结果 |
|---|---|---|
| 1080p YUV420 | 1920×1080×1.5 | 3.1 MB/帧；@30fps ≈ **93 MB/s ≈ 335 GB/h** |
| 1080p 灰度（特征提取输入） | 1920×1080×1 | 2 MB/帧；@30fps ≈ 62 MB/s |
| IMU（200Hz × 6 值 float64） | 200×6×8 | ~9.6 KB/s（可忽略） |
| 磁强计（100Hz × 6 值） | 100×6×8 | ~4.8 KB/s（可忽略） |
| GNSS（1Hz 全卫星测量） | 每批次 ~10–30 条 × ~200B | ~2–6 KB/s（可忽略） |

- 结论：**带宽与存储瓶颈在图像**。长时间采集必须压缩（H.264/HEVC 编码）或降分辨率/降帧率；8GB 内存中应用进程可用内存有限，原始 YUV 缓冲队列需控制长度。

## 5. 算力与功耗预算（平台事实）

- SD778G 可用计算资源：2 个大核 A78（2.4GHz/2.2GHz）+ 4 小核 A55；峰值 12 TOPS 的 Hexagon 770 可承担部分算子（成熟度需实测）。
- 30fps 视频下每帧预算约 33ms（全管线，含采集/编码/特征提取/融合），长时间运行需预留温控降频余量。
- 全传感器开启 + 相机 + 计算时功耗较高：4000mAh 电池，实测续航有限；磁强计读数受充电电流干扰，采集期间不宜充电。
- 无散热风扇，持续满载会触发降频，实时性设计需留 30–50% 余量（推荐）。

## 6. 环境约束（事实）

| 约束 | 说明 |
|---|---|
| GNSS | 需室外开阔天空；室内/隧道无观测 |
| 磁强计 | 靠近金属/强磁体（钢筋、车辆、电器）读数严重退化 |
| 光照 | 主摄 f/1.9 无 OIS，暗光下曝光变长、运动模糊增加 |
| 电池/温控 | 长时间采集发热降频，建议分段落采集 |
| 系统限制 | Magic UI 后台杀进程，需前台服务；Android 11 无传感器后台节流 |

## 7. 采集日志必须包含的信息（事实清单）

- 每路数据原始时间戳（不换算、不丢失）；
- 相机：帧时间戳、曝光时间、ISO、对焦距离、AE/AF 状态、分辨率；
- 传感器：UNCALIBRATED 六元组（值 + 偏差）、事件时间戳；
- GNSS：GnssClock 全字段、每条测量（伪距/相位/多普勒/载噪比/星座/频率）、Location（经纬度/海拔/速度/精度/时间）；
- 运行环境：系统版本、传感器清单（名称/厂商/最高速率）、相机能力（输出尺寸/RAW 可用性/Gnss 双频）、电池与温度采样。

## 参考来源

- Android 传感器时间戳与坐标：https://developer.android.com/develop/sensors-and-location/sensors/sensors_motion
- Android Camera2 时间戳：https://developer.android.com/media/camera/camera2
- Android GNSS：https://developer.android.com/develop/sensors-and-location/sensors/gnss
- Qualcomm Snapdragon 778G 产品简报：https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/Snapdragon-778G-5G-Product-Brief.pdf
