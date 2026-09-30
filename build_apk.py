#!/usr/bin/env python3
"""APK Build Assistant - cellphone-gvio

功能：
  1. 检测 backend/third_party 下的第三方库是否存在（文件存在性 + 最小化编译测试）。
     若缺失，则通过 `git clone --depth 1 ...` 等方式下载并按需准备/编译。
  2. 交互式询问版本号（默认 version code 为当前 Unix 时间戳），写入 build.gradle。
  3. 调用 Gradle 构建 debug APK，并复制到项目根目录 gvio-sensor-viewer.apk。

用法：
    python build_apk.py [version_code [version_name]] [--no-deps] [--deps-only]

示例：
    python build_apk.py                 # 交互式，自动准备依赖并构建
    python build_apk.py 4 1.2.0         # 指定版本
    python build_apk.py --deps-only     # 只检测/准备第三方库
    python build_apk.py --no-deps       # 跳过第三方库检测
"""

import os
import re
import sys
import time
import shutil
import tempfile
import argparse
import subprocess

# ---------------------------------------------------------------------------
# 项目路径
# ---------------------------------------------------------------------------
PROJECT_ROOT = r"E:\Development\Python_Projects\cellphone-gvio"
ANDROID_APP = os.path.join(PROJECT_ROOT, "android-app")
BACKEND = os.path.join(PROJECT_ROOT, "backend")
THIRD_PARTY = os.path.join(BACKEND, "third_party")

# Gradle / JDK
GRADLE_BAT = r"E:\AppData\Gradle\gradle-6.7.1\bin\gradle.bat"
GRADLE_HOME = r"C:\Users\ADMINI~1\AppData\Local\Temp\opencode\gradle-home"


# ===========================================================================
# 通用工具
# ===========================================================================
def log(msg):
    print(msg)
    sys.stdout.flush()


def _which(prog):
    from shutil import which
    return which(prog)


def _run(cmd, cwd=None, timeout=600, env=None):
    try:
        return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                              timeout=timeout, env=env)
    except Exception as exc:  # noqa: BLE001
        log("  command error: " + str(exc))
        return None


def try_compile(source, include_dirs, lang="c++", timeout=90):
    """最小化编译测试。

    返回 True（编译通过）/ False（有编译器但编译失败）/ None（没有可用编译器）。
    使用 -fsyntax-only（gcc/clang）或 /Zs（MSVC），不产生目标文件。
    """
    tmp = tempfile.mkdtemp(prefix="gvio_probe_")
    ext = ".cpp" if lang == "c++" else ".c"
    src = os.path.join(tmp, "probe" + ext)
    try:
        with open(src, "w", encoding="utf-8") as fh:
            fh.write(source)
    except OSError:
        shutil.rmtree(tmp, ignore_errors=True)
        return None

    if lang == "c++":
        candidates = ["g++", "clang++", "cl"]
    else:
        candidates = ["gcc", "clang", "cl"]

    saw_compiler = False
    for cc in candidates:
        path = _which(cc)
        if not path:
            continue
        saw_compiler = True
        if os.path.basename(cc) == "cl":
            cmd = [path, "/nologo", "/Zs",
                   ("/TP" if lang == "c++" else "/TC")]
            cmd += ["/I" + d for d in include_dirs]
            cmd += [src]
        else:
            cmd = [path, "-fsyntax-only"]
            cmd += ["-I" + d for d in include_dirs]
            cmd += ["-x", "c++" if lang == "c++" else "c"]
            cmd += [src]
        res = _run(cmd, timeout=timeout)
        if res is not None and res.returncode == 0:
            shutil.rmtree(tmp, ignore_errors=True)
            return True

    shutil.rmtree(tmp, ignore_errors=True)
    return False if saw_compiler else None


def git_clone(url, dest, ref=None, depth=1, timeout=900):
    """git clone --depth <depth> [--branch ref] url dest（禁止交互）。"""
    env = os.environ.copy()
    env["GIT_TERMINAL_PROMPT"] = "0"      # 不要卡在账号/密码提示
    env["GIT_ASKPASS"] = "echo"
    cmd = ["git", "clone", "--depth", str(depth)]
    if ref:
        cmd += ["--branch", ref]
    cmd += [url, dest]
    log("  $ " + " ".join(cmd))
    res = _run(cmd, timeout=timeout, env=env)
    if res is None:
        return False
    if res.returncode != 0:
        tail = (res.stderr or res.stdout or "")[-1200:]
        log("  git clone failed:\n" + tail)
        return False
    return True


def clone_any(urls, dest, ref=None):
    """依次尝试多个 URL，带 ref 失败后回退到默认分支。"""
    for url in urls:
        if ref and git_clone(url, dest, ref):
            return True
        # 清掉可能残留的空目录
        if os.path.isdir(dest):
            shutil.rmtree(dest, ignore_errors=True)
        os.makedirs(dest, exist_ok=True)
        if git_clone(url, dest):
            return True
        if os.path.isdir(dest):
            shutil.rmtree(dest, ignore_errors=True)
    return False


# ===========================================================================
# Eigen (header-only, MPL-2.0)
# ===========================================================================
def detect_eigen():
    for base in ("eigen", "Eigen"):
        marker = os.path.join(THIRD_PARTY, base, "src", "Core", "util", "Macros.h")
        if os.path.isfile(marker):
            inc = os.path.join(THIRD_PARTY, base)
            # 代码以 <Eigen/Core> 引用，include 根为 third_party
            test = ("#include <Eigen/Core>\n#include <Eigen/Geometry>\n"
                    "int main(){ Eigen::Matrix3d m = Eigen::Matrix3d::Identity();"
                    " return (int)m(0,0) - 1; }\n")
            return _finish(detect_eigen, "Eigen",
                           try_compile(test, [THIRD_PARTY], "c++"), marker)
    return False, "not found"


def fetch_eigen():
    tmp = tempfile.mkdtemp(prefix="gvio_eigen_")
    ok = clone_any(["https://gitlab.com/libeigen/eigen.git",
                    "https://github.com/libigl/eigen.git"],
                   tmp, ref="3.4.0")
    if not ok:
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    src = os.path.join(tmp, "Eigen")
    if not os.path.isdir(src):
        log("  unexpected Eigen layout (no 'Eigen' folder)")
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    dst = os.path.join(THIRD_PARTY, "eigen")
    if os.path.isdir(dst):
        shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)
    shutil.rmtree(tmp, ignore_errors=True)
    return True


# ===========================================================================
# nlohmann/json (single header, MIT)
# ===========================================================================
def detect_nlohmann():
    marker = os.path.join(THIRD_PARTY, "nlohmann", "json.hpp")
    if os.path.isfile(marker):
        test = ("#include <nlohmann/json.hpp>\n"
                "int main(){ nlohmann::json j; j[\"a\"] = 1; return 0; }\n")
        return _finish(detect_nlohmann, "nlohmann/json",
                       try_compile(test, [THIRD_PARTY], "c++"), marker)
    return False, "not found"


def fetch_nlohmann():
    tmp = tempfile.mkdtemp(prefix="gvio_json_")
    ok = clone_any(["https://github.com/nlohmann/json.git"], tmp, ref="v3.11.3")
    if not ok:
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    src = os.path.join(tmp, "single_include", "nlohmann", "json.hpp")
    if not os.path.isfile(src):
        log("  unexpected nlohmann layout")
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    dst_dir = os.path.join(THIRD_PARTY, "nlohmann")
    os.makedirs(dst_dir, exist_ok=True)
    shutil.copy2(src, os.path.join(dst_dir, "json.hpp"))
    shutil.rmtree(tmp, ignore_errors=True)
    return True


# ===========================================================================
# mbedTLS (Apache-2.0 OR GPL-2.0-or-later；此处按 Apache-2.0 使用)
# ===========================================================================
def detect_mbedtls():
    hdr = os.path.join(THIRD_PARTY, "mbedtls", "include", "mbedtls", "ssl.h")
    ssl = os.path.join(THIRD_PARTY, "mbedtls", "library", "ssl_tls.c")
    if os.path.isfile(hdr) and os.path.isfile(ssl):
        test = ("#include <mbedtls/ssl.h>\n#include <mbedtls/entropy.h>\n"
                "int main(void){ return 0; }\n")
        return _finish(detect_mbedtls, "mbedTLS",
                       try_compile(test, [os.path.join(THIRD_PARTY, "mbedtls", "include")], "c"),
                       hdr)
    return False, "not found"


def fetch_mbedtls():
    tmp = tempfile.mkdtemp(prefix="gvio_mbedtls_")
    ok = clone_any(["https://github.com/Mbed-TLS/mbedtls.git"], tmp,
                   ref="mbedtls-3.6.4")
    if not ok:
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    dst = os.path.join(THIRD_PARTY, "mbedtls")
    if os.path.isdir(dst):
        shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(tmp, dst)
    shutil.rmtree(tmp, ignore_errors=True)
    return True


# ===========================================================================
# nghttp2 (MIT, vendored as flat lib sources + public includes)
# ===========================================================================
def detect_nghttp2():
    hdr = os.path.join(THIRD_PARTY, "nghttp2", "includes", "nghttp2", "nghttp2.h")
    src = os.path.join(THIRD_PARTY, "nghttp2", "nghttp2_session.c")
    if os.path.isfile(hdr) and os.path.isfile(src):
        test = "#include <nghttp2/nghttp2.h>\nint main(void){ return 0; }\n"
        return _finish(detect_nghttp2, "nghttp2",
                       try_compile(test, [os.path.join(THIRD_PARTY, "nghttp2", "includes")], "c"),
                       hdr)
    return False, "not found"


def _ensure_nghttp2ver(includedir, version):
    out = os.path.join(includedir, "nghttp2ver.h")
    if os.path.isfile(out):
        return
    parts = (version.split(".") + ["0", "0", "0"])[:3]
    try:
        major, minor, patch = (int(parts[0]), int(parts[1]), int(parts[2]))
    except ValueError:
        major, minor, patch = 0, 0, 0
    num = (major << 16) | (minor << 8) | patch
    tmpl = os.path.join(includedir, "nghttp2ver.h.in")
    if os.path.isfile(tmpl):
        with open(tmpl, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        text = text.replace("@PACKAGE_VERSION@", version)
        text = text.replace("@PACKAGE_VERSION_NUM@", "%06x" % num)
        text = text.replace("@PACKAGE_VERSION_MAJOR@", str(major))
        text = text.replace("@PACKAGE_VERSION_MINOR@", str(minor))
        text = text.replace("@PACKAGE_VERSION_PATCH@", str(patch))
        text = re.sub(r"@[A-Za-z_]+@", "0", text)
        with open(out, "w", encoding="utf-8") as fh:
            fh.write(text)
        return
    with open(out, "w", encoding="utf-8") as fh:
        fh.write('#define NGHTTP2_VERSION "%s"\n' % version)
        fh.write("#define NGHTTP2_VERSION_NUM 0x%06x\n" % num)


def fetch_nghttp2():
    ref = "v1.62.1"
    version = ref.lstrip("v")
    tmp = tempfile.mkdtemp(prefix="gvio_nghttp2_")
    ok = clone_any(["https://github.com/nghttp2/nghttp2.git"], tmp, ref=ref)
    if not ok:
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    lib = os.path.join(tmp, "lib")
    if not os.path.isdir(lib):
        log("  unexpected nghttp2 layout (no 'lib' folder)")
        shutil.rmtree(tmp, ignore_errors=True)
        return False
    dst = os.path.join(THIRD_PARTY, "nghttp2")
    if os.path.isdir(dst):
        shutil.rmtree(dst, ignore_errors=True)
    os.makedirs(dst)
    for name in os.listdir(lib):
        full = os.path.join(lib, name)
        if os.path.isfile(full) and (name.endswith(".c") or name.endswith(".h")):
            shutil.copy2(full, dst)
    inc_src = os.path.join(lib, "includes")
    if os.path.isdir(inc_src):
        shutil.copytree(inc_src, os.path.join(dst, "includes"))
    _ensure_nghttp2ver(os.path.join(dst, "includes", "nghttp2"), version)
    shutil.rmtree(tmp, ignore_errors=True)
    return True


# ===========================================================================
# 依赖清单
# ===========================================================================
DEPS = [
    {"name": "eigen",    "detect": detect_eigen,    "fetch": fetch_eigen},
    {"name": "mbedtls",  "detect": detect_mbedtls,  "fetch": fetch_mbedtls},
    {"name": "nghttp2",  "detect": detect_nghttp2,  "fetch": fetch_nghttp2},
    {"name": "nlohmann", "detect": detect_nlohmann, "fetch": fetch_nlohmann},
]


def _finish(fn, label, compile_result, marker):
    """把“文件存在 + 编译测试”结果整理成 (ok, detail)。"""
    if compile_result is True:
        return True, "present, compile test passed (" + marker + ")"
    if compile_result is None:
        return True, "present, compile test skipped (no compiler) (" + marker + ")"
    return False, "files present but minimal compile test FAILED (" + marker + ")"


def ensure_third_party():
    log("== Third-party dependency check ==")
    os.makedirs(THIRD_PARTY, exist_ok=True)
    all_ok = True
    for dep in DEPS:
        ok, detail = dep["detect"]()
        if ok:
            log("  [OK]   %-9s %s" % (dep["name"], detail))
            continue
        log("  [MISS] %-9s %s" % (dep["name"], detail))
        log("         fetching via git clone --depth 1 ...")
        if dep["fetch"]():
            ok2, detail2 = dep["detect"]()
            if ok2:
                log("  [OK]   %-9s fetched: %s" % (dep["name"], detail2))
            else:
                all_ok = False
                log("  [WARN] %-9s fetched but verification failed: %s"
                    % (dep["name"], detail2))
        else:
            all_ok = False
            log("  [FAIL] %-9s could not be obtained (offline?)" % dep["name"])
    log("")
    return all_ok


# ===========================================================================
# 版本号修改
# ===========================================================================
def ask(prompt, default):
    try:
        val = input(prompt).strip()
        return val if val else default
    except EOFError:
        return default


def update_version_code_name(code, name):
    gradle_path = os.path.join(ANDROID_APP, "app", "build.gradle")
    if not os.path.isfile(gradle_path):
        log("Not found " + gradle_path + ", skip version update.")
        return
    with open(gradle_path, "r", encoding="utf-8", errors="replace") as fh:
        content = fh.read()
    content = re.sub(r"versionCode\s+\d+", "versionCode " + str(code), content)
    content = re.sub(r"versionName\s+'[^']*'", "versionName '" + name + "'", content)
    with open(gradle_path, "w", encoding="utf-8") as fh:
        fh.write(content)
    log("OK " + gradle_path + " updated: versionCode=" + str(code)
        + ", versionName='" + name + "'")


# ===========================================================================
# Gradle 构建
# ===========================================================================
def clear_gradle_cache():
    try:
        subprocess.run([GRADLE_BAT, "--stop", "-g", GRADLE_HOME],
                       cwd=ANDROID_APP, capture_output=True, text=True, timeout=30)
    except Exception:
        pass
    cache_dir = os.path.join(GRADLE_HOME, "caches")
    if os.path.isdir(cache_dir):
        shutil.rmtree(cache_dir)
        log("Gradle cache cleared")


def build_assemble_debug():
    clear_gradle_cache()
    cmd = [GRADLE_BAT, "clean", "assembleDebug", "--no-daemon",
           "-g", GRADLE_HOME, "--no-parallel"]
    log("Running Gradle clean assembleDebug ...")
    res = _run(cmd, cwd=ANDROID_APP, timeout=900)
    if res is None:
        return False
    if res.returncode == 0:
        log("BUILD SUCCESSFUL")
        return True
    log("BUILD FAILED")
    out = (res.stdout or "")[-4000:]
    err = (res.stderr or "")[-4000:]
    if out.strip():
        log("   stdout:\n" + out)
    if err.strip():
        log("   stderr:\n" + err)
    return False


def copy_debug_apk():
    src = os.path.join(ANDROID_APP, "app", "build", "outputs", "apk", "debug", "app-debug.apk")
    dst = os.path.join(PROJECT_ROOT, "gvio-sensor-viewer.apk")
    if not os.path.isfile(src):
        log("Source APK not found: " + src)
        return False
    shutil.copy2(src, dst)
    log("APK copied to project root: " + dst + " (" + str(os.path.getsize(dst)) + " bytes)")
    return True


# ===========================================================================
# 主入口
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description="Build APK for cellphone-gvio")
    parser.add_argument("version_code", nargs="?", default=None, type=int,
                        help="Version code (integer, default = Unix timestamp)")
    parser.add_argument("version_name", nargs="?", default=None,
                        help="Version name (string, e.g., 0.1.0)")
    parser.add_argument("--no-deps", action="store_true",
                        help="Skip third-party dependency check")
    parser.add_argument("--deps-only", action="store_true",
                        help="Only ensure third-party dependencies, then exit")
    args = parser.parse_args()

    # ---- 第三方依赖 ----
    if not args.no_deps:
        ensure_third_party()
    if args.deps_only:
        return

    # ---- 版本号 ----
    vc = args.version_code
    vn = args.version_name
    if vc is None:
        vc = int(time.time())
        log("Using default version code (Unix timestamp): " + str(vc))
    if vn is None:
        vn = ask("Enter version name (e.g., 0.1.0, default 0.1.0): ", "0.1.0")
    try:
        version_code = int(vc)
    except (TypeError, ValueError):
        version_code = int(time.time())
    update_version_code_name(version_code, vn)

    # ---- 构建 & 导出 ----
    if build_assemble_debug():
        copy_debug_apk()
        log("\nDone! APK ready at: "
            + os.path.abspath(os.path.join(PROJECT_ROOT, "gvio-sensor-viewer.apk")))
    else:
        log("\nBuild failed, please check the errors above and fix.")


if __name__ == "__main__":
    main()
