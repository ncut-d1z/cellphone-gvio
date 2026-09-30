# 01 操作系统平台（Android 11 / Magic UI 4.2）

> 软件篇 · 系统与运行环境
> 整理时间：2026-08

## 1. 系统版本

| 项目 | 参数 |
|---|---|
| 出厂系统 | Android 11（API level 30）+ Magic UI 4.2 |
| 内核 | Android 11 标准内核（Kernel 5.x 系，具体版本以系统设置为准） |
| Google 服务 | 有 GMS 版本（市场/区域相关）；国行无 Google Play 服务 |
| 更新 | 官方承诺 3 年软件更新 + 5 年安全更新；截至 2026 年软件支持期已结束 |
| 升级 | 部分市场可升级至 Magic UI 6.x（Android 12），以真机为准（**待实测**） |

> 采集与融合模块应基于 API 30 编译（`targetSdkVersion 30` 或兼容此版本），避免依赖 Android 12+ 新增 API。

## 2. 对本项目相关的系统能力（API 30 事实）

| 能力 | 说明 |
|---|---|
| Camera2 API | 完全支持（API 21+），可枚举全部相机能力 |
| SensorManager | 完全支持，含 UNCALIBRATED 变体与 `CLOCK_MONOTONIC` 时间戳 |
| GNSS 原始观测量 | `GnssMeasurement`（API 24+）可用，是否开放依赖厂商 HAL（**待实测**） |
| 前台服务 | 持续后台采集需前台服务 + 通知（Android 8+ 要求） |
| 后台限制 | Android 11 未引入传感器后台节流（Android 12 起有），但仍受 Doze 影响，采集应保持屏幕/前台服务 |
| 定位 | 需要 `ACCESS_FINE_LOCATION` 权限 + 开启定位服务 |
| HTTP/2 | 系统网络栈（OkHttp/Conscrypt）支持 HTTP/2（h2 over TLS），本地回环通信亦可 |

## 3. 权限清单（本项目采集所需）

| 权限 | 用途 |
|---|---|
| `android.permission.CAMERA` | Camera2 视频/图像采集 |
| `android.permission.BODY_SENSORS` | 加速度计/陀螺仪（Android 10+ 开始作为身体传感器） |
| `android.permission.ACCESS_FINE_LOCATION` | GNSS 位置与原始观测量 |
| `android.permission.ACCESS_COARSE_LOCATION` | 网络定位辅助（可选） |
| `android.permission.FOREGROUND_SERVICE` | 前台服务持续采集（可选但推荐） |

> `BODY_SENSORS` 为运行时权限（Android 10+），部分厂商在 Android 11 将加速度计/陀螺仪划入身体传感器范畴，必须动态申请。

## 4. 厂商系统特性（对采集的影响，事实层面）

- Magic UI 对后台应用存在激进杀进程策略：长时间采集需前台服务 + 常驻通知，必要时在系统"应用启动管理"中允许自启动/后台运行。
- 系统温控会降频并可能限制相机持续高分辨率预览时长。
- 无 root、无 GMS 的国行版本无法使用部分 Google 定位辅助数据服务；纯 GNSS 定位不受影响。

## 5. 工程约束小结

- 目标平台固定为这一台真机：所有"待实测"项直接在真机验证，避免跨机型兼容负担。
- 采集 App 建议 `targetSdk 30`（或 31+），并在真机上确认无厂商限制。

## 参考来源

- GSMArena（Android 11 / Magic UI 4.2）：https://www.gsmarena.com/honor_50_pro-10958.php
- Beebom（系统与更新周期）：https://gadgets.beebom.com/mobile/honor-50-pro
- Android 11 行为变更：https://developer.android.com/about/versions/11/behavior-changes-11
