# 01 整机规格（HONOR 50 Pro）

> 硬件篇 · 设备总体规格
> 整理时间：2026-08

## 1. 基本信息

| 项目 | 参数 |
|---|---|
| 品牌/机型 | HONOR 50 Pro |
| 型号代码 | HN2RNAM |
| 发布 | 2021-06-16（中国）/ 2021-06-25（全球） |
| 尺寸 | 163.5 × 74.7 × 8.05 mm |
| 重量 | 187 g |
| 颜色 | 初雪水晶、墨玉青、夏日琥珀、亮黑 |
| 首发价格（国行） | ¥3699（8GB + 256GB） |
| 防水 | 无防水等级认证 |

## 2. 屏幕

| 项目 | 参数 |
|---|---|
| 尺寸 | 6.72 英寸 |
| 类型 | OLED（曲面屏） |
| 分辨率 | 1236 × 2676（FHD+） |
| 刷新率 | 120 Hz |
| 像素密度 | 439 PPI |
| 宽高比 | 19.5:9 |

## 3. 处理器与内存

| 项目 | 参数 |
|---|---|
| SoC | Qualcomm Snapdragon 778G 5G（SM7325） |
| CPU | Kryo 670（1×2.4GHz A78 + 3×2.2GHz A78 + 4×1.9GHz A55） |
| GPU | Adreno 642L |
| 运行内存 | 8GB / 12GB（有资料标注 3200MHz，对应 LPDDR5，待实测确认） |
| 存储 | 256GB，不支持 microSD 扩展 |

> 详见 [02-soc-snapdragon-778g.md](02-soc-snapdragon-778g.md)。

## 4. 电池与充电

| 项目 | 参数 |
|---|---|
| 电池 | 4000 mAh Li-Po（不可拆卸） |
| 有线快充 | 100W 超级快充（宣传 25 分钟充满） |
| 无线充电 | 不支持 |
| 接口 | USB Type-C 2.0，支持 OTG |

## 5. 网络与连接

| 项目 | 参数 |
|---|---|
| 制式 | 5G / 4G LTE-A / 3G / 2G，双 Nano-SIM 双待 |
| 5G 频段 | n1 / n5 / n8 / n41 / n77 / n78（SA/NSA） |
| 4G 频段 | 1 / 2 / 4 / 5 / 8 / 19 / 34 / 38 / 39 / 40 / 41 |
| Wi-Fi | Wi-Fi 6（802.11 a/b/g/n/ac/ax），双频 2.4GHz + 5GHz |
| 蓝牙 | 5.2（AAC / aptX / aptX HD / LDAC / SBC / LE） |
| NFC | 支持（多数资料）；个别资料标注不支持，待实测 |
| 定位 | GPS、A-GPS、GLONASS、BDS、Galileo（详见 GNSS 篇） |

> 注意：SoC 的 FastConnect 6700 硬件支持 Wi-Fi 6E（6GHz），但该机型宣传与资料均为双频 Wi-Fi 6，6GHz 频段未启用。

## 6. 传感器清单（出厂）

| 传感器 | 是否配备 | 备注 |
|---|---|---|
| 加速度计 | 有 | 重力传感器 |
| 陀螺仪 | 有 | 与加速度计常为同一 IMU 芯片 |
| 磁强计（罗盘） | 有 | 独立磁力计芯片 |
| 距离传感器 | 有 | 通话时熄屏 |
| 环境光传感器 | 有 | 自动亮度 |
| 屏下指纹（光学） | 有 | 指纹识别 |
| 气压计 | **无** | 无海拔参考，高度只能依靠 GNSS / 视觉估计 |

## 7. 对本项目的硬件要点

- 无气压计：垂直方向（高度）无直接气压量测。
- 无 OIS（光学防抖），仅有 EIS（电子防抖，主要在相机 App 视频链路）。
- 传感器芯片具体型号未公开，需运行时检测（见 [04-imu-magnetometer.md](04-imu-magnetometer.md)）。
- 双 Nano-SIM 双待，任一卡位插卡即可获得蜂窝网络辅助定位（A-GPS）。

## 参考来源

- GSMArena：https://www.gsmarena.com/honor_50_pro-10958.php
- GSMChina：https://gsmchina.com/smartphones/honor-50-pro
- Gizmochina：https://www.gizmochina.com/product/honor-50-pro
- Beebom：https://gadgets.beebom.com/mobile/honor-50-pro
- PhoneBunch：https://www.phonebunch.com/phone/honor-50-pro-4342
- PhoneArena：https://www.phonearena.com/phones/Honor-50-Pro_id11757
