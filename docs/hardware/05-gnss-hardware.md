# 05 GNSS 硬件

> 硬件篇 · 卫星定位
> 整理时间：2026-08

## 1. 能力分层

GNSS 能力分为**芯片平台能力**（Snapdragon 778G）与**整机实际开放能力**（HONOR 50 Pro 的 HAL/天线/固件）两层：

### 平台能力（SoC 规格）

| 能力 | 参数 |
|---|---|
| 星座 | GPS、GLONASS、BeiDou、Galileo、QZSS、NavIC |
| 频率 | **L1/L5 双频** |
| 定位增强 | Sidewalk/Lane-level Positioning（人行道/车道级）、Sensor-Assisted Positioning（传感器辅助定位） |
| 蜂窝辅助 | X53 调制解调器支持 A-GNSS 辅助数据 |

### 整机宣传规格

| 来源 | 标注 |
|---|---|
| 多家规格站 | GPS、A-GPS、GLONASS、BDS（部分站点增列 Galileo） |
| 定位模式 | 网络定位 + 卫星定位（GPS/AGPS） |

## 2. 需要实测验证的关键点

- **L5 频段是否开放**：SoC 支持双频，但整机是否向第三方 App 暴露 L5 原始观测量取决于厂商 HAL 配置。验证方法：`GnssCapabilities.hasMeasurementCorrections()` 之外，直接检查 `GnssMeasurement` 中 `CarrierFrequencyHz` 是否出现 1176.45MHz（L5）/ 1575.42MHz（L1）以及 `ConstellationType` 分布（见 [软件篇 04](../software/04-gnss-api.md)）。
- **原始观测量是否可用**：`GnssMeasurement` 回调是否持续产出伪距/载波相位，取决于 HAL 实现（Android 7+ 标准 API，厂商可能禁用）。
- **双频精度预期**：若 L1+L5 双频开放，市区/多路径环境下位置精度通常优于单频（量级上单频典型 5–10m，双频城市环境更优）；若仅单频，精度受多路径影响明显。

## 3. 整机层面的客观约束（硬件事实）

| 事实 | 说明 |
|---|---|
| 天线 | 手机内建天线，无外接 GNSS 天线接口；户外开阔天空下效果最佳 |
| 更新率 | 消费级手机典型 1Hz 位置更新 |
| 冷启动 | 首次定位（冷启动）通常需要数十秒，热启动更快 |
| 遮挡 | 室内、地下、隧道、高架桥下无可用卫星；城市峡谷多路径严重 |
| 电池 | 定位 + 相机 + IMU 全开功耗较大，长时间采集注意电量与发热 |

## 4. 对项目的数据接口事实

| 事实 | 说明 |
|---|---|
| 坐标基准 | 位置输出为 WGS84 地理坐标（经纬度 + 海拔） |
| 速度 | Android 位置还提供速度（地面速度 m/s）与方位（bearing） |
| 精度 | 位置附带水平精度估计（accuracy，米）；原始观测量阶段另有载噪比等字段 |
| 时间 | 卫星时间使用 GPS 时系（TOW/周数），与手机本地时钟不同（见 [软件篇 04](../software/04-gnss-api.md)） |
| 与 IMU/相机 | 三者时间基准不同域，采集时需记录各自时间戳并留出对齐所需信息 |

## 参考来源

- Qualcomm Snapdragon 778G 5G 产品简报（L1/L5、Sensor-Assisted Positioning）：https://www.qualcomm.com/content/dam/qcomm-martech/dm-assets/documents/Snapdragon-778G-5G-Product-Brief.pdf
- GSMChina（整机定位规格）：https://gsmchina.com/smartphones/honor-50-pro
- Beebom（整机定位规格）：https://gadgets.beebom.com/mobile/honor-50-pro
- Android GNSS 能力参考：https://developer.android.com/develop/sensors-and-location/sensors/gnss
