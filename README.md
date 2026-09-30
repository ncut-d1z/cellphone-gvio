# cellphone-gvio

面向手机单目视觉、IMU、磁强计与 GNSS 融合的 MSCKF 研究原型。

**当前 APK 仍是 Android 传感器／相机预览器，未接通 C++ 融合引擎。** `android-app/legacy-java/` 默认不参与编译；`frontend/` 仍主要是第三方 JavaScript 库，并不是已经完成的轨迹页面。后端的单元／集成回归不等于手机实机导航精度已经验证。

## 安全：先更换旧密钥

原先提交的两份 `server.key` 及配套证书已从当前版本移除。**旧密钥必须视为已公开，不得继续使用。** 本次没有重写 Git 历史，历史版本、其他克隆和旧 APK 仍可能包含旧密钥。

在仓库根目录生成仅供本机开发使用的密钥与自签名证书：

```powershell
.\generate_private_key.ps1
# 显式指定 OpenSSL，或增加开发用主机名／IP
.\generate_private_key.ps1 -OpenSSLPath 'D:\Tools\OpenSSL\bin\openssl.exe' -DnsNames localhost,my-pc -IpAddresses 127.0.0.1,192.168.1.10
# 已存在时默认拒绝覆盖；明确轮换才加 -Force
.\generate_private_key.ps1 -Force
```

默认输出 `backend/certs/server.key` 和 `backend/certs/server.crt`，不会自动信任证书，也不会复制私钥到 APK。Windows PowerShell 5.1+；需要 OpenSSL，可从 PATH 或 Git for Windows 自动查找。支持 `-OutputDirectory`、`-KeyBits`、`-Days` 等参数，见脚本内帮助。

`.gitignore` 排除密钥及签名容器，CI 的 `tools/check_no_private_keys.py` 同时检查 **Git 暂存区内容**，包括改名后的 PEM 私钥。启用本机提交前检查：

```powershell
git config core.hooksPath .githooks
python tools/check_no_private_keys.py
```

忽略规则和本机 hook 可被绕过，CI 是提交后的检查；如需合并前强制阻断，应将私钥检查设为受保护分支的必需检查。本项目没有擅自修改仓库分支保护规则。

## 目录与职责

| 路径 | 用途 |
|---|---|
| `android-app/` | Java / Camera2 / SensorManager / GNSS 预览程序 |
| `backend/include/gvio/`、`backend/src/` | 数学核心、时间调度、融合引擎与可选 TLS 服务 |
| `backend/tests/` | 核心、生命周期和真实 HTTP/2 + TLS + SSE 回归 |
| `docs/P0_FIXES.md` | P0 修复约定、行为变化、测试范围与后续边界 |
| `docs/` | 原有 HONOR 50 Pro 设备知识库 |
| `tools/`、`.githooks/` | 私钥检查与生成脚本回归 |
| `frontend/` | 现有第三方前端库；完整页面仍待实现 |

## C++ 后端：先无网络运行

需要 C++17 编译器、CMake 3.16+、Eigen 与 nlohmann/json。**默认不构建也不启动网络服务，不需要任何证书。** 不再通过 APK 构建脚本准备后端依赖；推荐使用系统包或 vcpkg 的完整依赖安装。

Ubuntu 示例：

```bash
sudo apt-get install cmake ninja-build clang libeigen3-dev nlohmann-json3-dev
cmake -S backend -B backend/build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGVIO_BUILD_TOOLS=ON
cmake --build backend/build --parallel 2
ctest --test-dir backend/build --output-on-failure
```

Windows + Visual Studio 2022 + vcpkg 示例（先把 `VCPKG_ROOT` 设置为本机安装目录）：

```powershell
& "$env:VCPKG_ROOT\vcpkg.exe" install eigen3:x64-windows nlohmann-json:x64-windows
cmake -S backend -B backend/build -A x64 "-DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" -DGVIO_BUILD_TOOLS=ON
cmake --build backend/build --config Debug --parallel 2
ctest --test-dir backend/build -C Debug --output-on-failure
```

MSVC 的大型 Eigen 对象已启用 `/bigobj`，并统一使用 `/utf-8`。已有合成数据工具 `gvio_desktop` 和 `gvio_isolate` 可继续编译；其输出是调试证据，不代表真实设备已经标定或达到导航精度要求。

仅测试数学、视觉和时间队列时，可加 `-DGVIO_CORE_ONLY=ON`，此时仅依赖 Eigen。GCC／Clang 内存与未定义行为检查可加 `-DGVIO_SANITIZE=ON`。

## 可选 HTTP/2 over TLS

编译开关与运行配置是两层：必须同时使用 `-DGVIO_WITH_SERVER=ON` 和 `FilterConfig::serverEnabled=true`。运行配置需提供新生成证书、私钥与 Web 根目录的路径。缺失证书或绑定失败时 `FusionEngine::start()` 返回 `false`，不会再假报启动成功。

Linux 上启用服务并运行集成测试：

```bash
sudo apt-get install libmbedtls-dev libnghttp2-dev openssl
python -m pip install 'httpx[http2]==0.28.1'
cmake -S backend -B backend/build -DGVIO_WITH_SERVER=ON
cmake --build backend/build --parallel 2
ctest --test-dir backend/build --output-on-failure
```

测试在临时目录生成并清理凭据，通过证书验证、ALPN、真实 HTTP/2、并行 SSE 和连接重建检查协议实现；不会关闭客户端证书验证。

| 路由 | 行为 |
|---|---|
| `GET /`、`GET /<path>` | 读取配置的 Web 根目录，首页须自行提供 |
| `GET /state` | `data: <JSON>\n\n` 格式的持续 SSE，约每 200 ms 发布 |
| `POST /control` | `{"cmd":"start|stop|reset|magcal"}`；投递控制命令后返回 `queued` |

HTTP `stop` 暂停并清空融合会话，但保留控制服务，避免服务器线程等待自身。HTTP `start` 开始新融合会话；真正终止工作线程与服务器应从外部调用 `FusionEngine::stop()`。服务定位为本机开发工具，默认绑定 `127.0.0.1`，尚未加入应用层鉴权，不应直接公开到公网。

## 时间与状态约定

全部融合入口须使用同一单调纳秒时间域。图像只有在 IMU 已精确传播到同一采集时刻时才能增广。融合层默认留出 **50 ms 有界重排序窗口**，线性插值 IMU 到观测时刻，记录过晚、超容量、非法或无 IMU 覆盖的观测；不做任意迟到量测的历史回滚。

`q` 将全局系向量旋到 IMU 系。右乘误差在全局系表达；物理陀螺传播为 `Exp(-omega_I * dt) * q`。完整索引与方程见 [P0 修复说明](docs/P0_FIXES.md)。

## Android 与后续工作

Android 工程和当前预览功能保留，未在此次修复中恢复 JNI。现有 `build_apk.py` 仍有本机路径配置（`PROJECT_ROOT`、`GRADLE_BAT`、`GRADLE_HOME`），使用前须修改；仅构建当前 APK 可用 `--no-deps` 跳过无关的 C++ 依赖准备。

后续仍需：实机采集／桌面回放、标定与时间源审查、Android 生命周期处理、GNSS 垂直精度与异常观测建模、滤波一致性评估、性能剖析与完整前端。不要未经审查直接恢复 `legacy-java/ClockSync.java` 的旧时钟换算。

CI 定义在 `.github/workflows/`。测试结果以对应提交的 Actions 运行记录为准。设备资料入口：[docs/README.md](docs/README.md)。第三方库许可由各上游项目规定；本次没有替项目选择或新增源码许可证。
