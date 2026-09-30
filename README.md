# cellphone-gvio

基于 **HONOR 50 Pro**（Snapdragon 778G，Android 11 / API 30）**单目视觉 + IMU + 磁强计 + GNSS**
的 **MSCKF** 紧耦合融合系统。系统由三部分组成：

- **`android-app/`** — Android 采集/预览 App（Camera2 双相机预览、Camera2/SensorManager/GNSS 采集、控制面板）。
- **`backend/`** — C++17 融合引擎（MSCKF、视觉跟踪、磁校准、WGS84↔ENU），内置 HTTP/2 over TLS 服务器。
- **`frontend/`** — Web 前端（Three.js），由后端服务器托管；当前仅包含第三方 JS 库。
- **`docs/`** — HONOR 50 Pro 软硬件知识库（硬件/软件篇）。
- **`build_apk.py`** — 项目构建脚本：准备第三方依赖 → 更新版本号 → 构建 APK。

---

## 目录结构

```
cellphone-gvio/
├── android-app/            # Android Studio / Gradle 工程 (com.gvio.app)
│   ├── app/                # 应用模块 (纯 Java，无 AndroidX)
│   ├── legacy-java/        # 早期 JNI 采集类（依赖 gvio_fusion .so，默认不参与编译）
│   ├── build.gradle
│   ├── settings.gradle
│   └── local.properties    # SDK 路径（本机生成，不提交）
├── backend/                # C++ 融合引擎
│   ├── CMakeLists.txt
│   ├── include/gvio/       # 公共头文件
│   ├── src/                # config/vision/msckf/fusion/server/tools
│   ├── certs/              # 自签名 TLS 证书 (server.crt / server.key)
│   └── third_party/        # 第三方库（按需下载，不提交，见下）
├── frontend/               # Web 前端 (Three.js)
├── docs/                   # 设备知识库
├── build_apk.py            # 一键构建脚本
├── .editorconfig
└── .gitignore
```

---

## 环境要求

### 通用

| 工具 | 版本 | 说明 |
|---|---|---|
| **Python** | 3.7+ | 运行 `build_apk.py` |
| **Git** | 任意较新版本 | 下载第三方库（`git clone --depth 1`） |
| **CMake** | ≥ 3.10 | 构建 C++ 后端 |
| **C/C++ 编译器** | 支持 **C++17** | MSVC 2017+/GCC 7+/Clang 5+；`gcc`/`g++`/`clang`/`cl` 任一 |

### Android 构建

| 工具 | 版本 | 说明 |
|---|---|---|
| **JDK** | **8**（或 11） | Gradle 6.7.1 需要 JDK 8/11；**不要用 JDK 17+**（会报 `Unsupported class file major version 65`） |
| **Android SDK** | `platforms;android-30`、`build-tools;30.0.3` | `compileSdk 30 / minSdk 30 / targetSdk 30` |
| **Gradle** | **6.7.1** | 或用 Android Studio 自带 Gradle |

> `android-app/local.properties` 需包含本机 SDK 路径，例如：
> ```properties
> sdk.dir=C\:\\Users\\<你>\\AppData\\Local\\Android\\Sdk
> ```
> 该文件已加入 `.gitignore`，需本机自行生成。

### 第三方库（Eigen / mbedTLS / nghttp2 / nlohmann）

这些库**不纳入源码仓库**（见 `.gitignore`），由 `build_apk.py` 检测并按需下载。也可手动执行：

```bash
python build_apk.py --deps-only
```

脚本对每个库同时做**文件存在性检查**与**最小化编译测试**（`g++/gcc -fsyntax-only`，无编译器时降级为文件检查）。缺失时通过 `git clone --depth 1 ...` 获取并校验：

| 库 | 版本 | 许可 |
|---|---|---|
| Eigen | 3.4.0 | MPL-2.0（少量文件 BSD） |
| mbedTLS | 3.6.x | Apache-2.0（本项目按 Apache-2.0 使用） |
| nghttp2 | 1.62.x | MIT |
| nlohmann/json | 3.11.3 | MIT |

---

## 构建与运行

### 1. 构建 Android APK

编辑 `build_apk.py` 顶部的本机路径常量（否则脚本会找错工具）：

```python
GRADLE_BAT  = r"…\gradle-6.7.1\bin\gradle.bat"
GRADLE_HOME = r"…\gradle-home"     # 独立 Gradle 用户目录
```

然后：

```bash
python build_apk.py                 # 准备依赖 → 交互式选版本 → 构建
python build_apk.py 4 1.2.0         # 指定 versionCode / versionName
python build_apk.py --no-deps       # 跳过第三方库检查
python build_apk.py --deps-only     # 只准备第三方库
```

`versionCode` 默认取**当前 Unix 时间戳**，保证每次构建唯一。成功后 APK 输出到项目根目录：

```
gvio-sensor-viewer.apk
```

安装到设备：

```bash
adb install -r gvio-sensor-viewer.apk
```

> APK 为**传感器预览器**：Camera2 双相机（后置+前置）等比预览、IMU/磁强计/GPS 实时显示，
> 以及相机控制面板（变焦 / 手动对焦 / 曝光补偿）。需要相机、位置、身体传感器权限。
> 注意：当前 APK **未链接** C++ 融合引擎（JNI/NDK 已移除，JNI 类在 `android-app/legacy-java/`）。

### 2. 构建 C++ 后端

先确保第三方库就绪（`python build_apk.py --deps-only`），然后：

```bash
cd backend
cmake -B build -DGVIO_BUILD_TOOLS=ON        # 生成构建系统
cmake --build build --config Release       # 编译
```

CMake 选项：

| 选项 | 默认 | 说明 |
|---|---|---|
| `GVIO_BUILD_TOOLS` | `OFF` | 构建桌面工具 `gvio_desktop`、`gvio_isolate` |
| `GVIO_ANDROID` | `OFF` | 构建 JNI 共享库（需 Android NDK，当前未启用） |

产物：静态库 `gvio_fusion`（+ 可选 `gvio_desktop` / `gvio_isolate`）。
Windows 下额外链接 `ws2_32`、`bcrypt`。

### 3. 运行后端工具

```bash
# 合成数据 1× 实时喂入，打印状态并对比真值；参数为时长(秒)
./build/gvio_desktop 60

# 分阶段喂数据，用于定位挂起/数值问题
./build/gvio_isolate
```

融合引擎会启动内嵌 HTTP/2 服务器（默认 `127.0.0.1:8443`，h2 over TLS，使用
`backend/certs/` 下的自签名证书），提供：

| 路由 | 说明 |
|---|---|
| `GET /` | 返回 `webRoot/index.html` |
| `GET /state` | SSE 状态流（`text/event-stream`） |
| `POST /control` | `{"cmd":"start|stop|reset|magcal"}` |
| `GET /<path>` | webRoot 静态文件 |

### 4. Web 前端

`frontend/` 目前仅包含 `three.min.js`、`OrbitControls.js`。将前端页面放入后端
`webRoot` 后即可通过 `https://<host>:8443/` 访问（自签名证书需在浏览器中信任）。

---

## 测试与调试

- **合成数据验证**：`gvio_desktop` / `gvio_isolate` 使用内置 `synthetic` 世界生成 IMU/磁/GNSS/图像真值。
- **逐步隔离**：`gvio_isolate` 按阶段喂入数据并打印时间戳，便于定位停滞或数值异常。
- **依赖自检**：`python build_apk.py --deps-only` 可随时验证第三方库是否完整、可编译。

---

## 许可证与第三方

- 本项目源码许可见 [`LICENSE`](LICENSE)（如尚未添加，请补充）。
- 第三方库均为宽松/文件级 copyleft 许可（MPL-2.0 / Apache-2.0 / MIT），**不强制**本项目开源，
  但分发源码或二进制时需**保留版权与许可声明**。建议维护一份 `THIRD_PARTY_NOTICES.md`。
- mbedTLS 为 **Apache-2.0 或 GPL-2.0-or-later** 双许可，本项目按 **Apache-2.0** 使用。

---

## 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| `Unsupported class file major version 65` | Gradle 使用了 JDK 17+。改用 **JDK 8/11**（设置 `JAVA_HOME`）。 |
| `Task 'assembleDebug' not found` | 在错误目录执行 Gradle。应在 `android-app/` 下执行。 |
| 第三方库缺失 / 后端编译找不到头文件 | 运行 `python build_apk.py --deps-only`。 |
| 最小化编译测试 `skipped (no compiler)` | 系统没有 `g++/gcc/clang/cl`，仅做了文件检查；请安装编译器。 |
| `git clone` 失败 | 需要网络/代理；脚本设置了 `GIT_TERMINAL_PROMPT=0`，不会卡在登录提示。 |

---

## 相关文档

- 设备知识库：[`docs/README.md`](docs/README.md)（HONOR 50 Pro 软硬件规格）
- Camera2 / Sensor / GNSS API 事实与数据接口记录：`docs/software/`
