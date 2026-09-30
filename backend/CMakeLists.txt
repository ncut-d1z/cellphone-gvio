cmake_minimum_required(VERSION 3.10)
project(gvio_fusion C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(GVIO_ANDROID "Build for Android (JNI)" OFF)
option(GVIO_BUILD_TOOLS "Build desktop tools/tests" OFF)

set(GVIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
  ${GVIO_ROOT}/include
  ${GVIO_ROOT}/third_party
  ${GVIO_ROOT}/third_party/nghttp2/includes
  ${GVIO_ROOT}/third_party/mbedtls/include
)

# ---------------------------------------------------------------------------
# nghttp2 (static, C)
# ---------------------------------------------------------------------------
file(GLOB NGHTTP2_SRC ${GVIO_ROOT}/third_party/nghttp2/*.c)
add_library(nghttp2 STATIC ${NGHTTP2_SRC})
target_include_directories(nghttp2 PUBLIC ${GVIO_ROOT}/third_party/nghttp2/includes)
target_compile_definitions(nghttp2 PUBLIC NGHTTP2_STATICLIB)
target_compile_options(nghttp2 PRIVATE -w)

# ---------------------------------------------------------------------------
# mbedtls (static, C)
# ---------------------------------------------------------------------------
file(GLOB MBEDTLS_SRC ${GVIO_ROOT}/third_party/mbedtls/library/*.c)
add_library(mbedtls STATIC ${MBEDTLS_SRC})
target_include_directories(mbedtls PUBLIC ${GVIO_ROOT}/third_party/mbedtls/include)
target_compile_options(mbedtls PRIVATE -w)

# ---------------------------------------------------------------------------
# gvio 后端本体
# ---------------------------------------------------------------------------
set(GVIO_SRC
  ${GVIO_ROOT}/src/config.cpp
  ${GVIO_ROOT}/src/vision.cpp
  ${GVIO_ROOT}/src/msckf.cpp
  ${GVIO_ROOT}/src/fusion.cpp
  ${GVIO_ROOT}/src/server/server.cpp
  ${GVIO_ROOT}/src/server/handler.cpp
)

add_library(gvio_fusion STATIC ${GVIO_SRC})
target_link_libraries(gvio_fusion PUBLIC nghttp2 mbedtls)
if(WIN32)
  target_link_libraries(gvio_fusion PUBLIC ws2_32 bcrypt)
endif()

# ---------------------------------------------------------------------------
# JNI 层(仅 Android)
# ---------------------------------------------------------------------------
if(GVIO_ANDROID)
  add_library(gvio_jni SHARED ${GVIO_ROOT}/src/jni/gvio_jni.cpp)
  target_link_libraries(gvio_jni PRIVATE gvio_fusion)
  set_target_properties(gvio_jni PROPERTIES OUTPUT_NAME gvio_fusion)
  target_link_libraries(gvio_jni PRIVATE log)
  find_library(LOG_LIB log)
endif()

# ---------------------------------------------------------------------------
# 桌面工具(验证/测试)
# ---------------------------------------------------------------------------
if(GVIO_BUILD_TOOLS)
  add_executable(gvio_desktop
    ${GVIO_ROOT}/src/tools/main_desktop.cpp
    ${GVIO_ROOT}/src/tools/synthetic.cpp
  )
  target_link_libraries(gvio_desktop PRIVATE gvio_fusion)
  target_compile_definitions(gvio_desktop PRIVATE GVIO_TOOL=1)

  add_executable(gvio_isolate
    ${GVIO_ROOT}/src/tools/test_isolate.cpp
    ${GVIO_ROOT}/src/tools/synthetic.cpp
  )
  target_link_libraries(gvio_isolate PRIVATE gvio_fusion)
  target_compile_definitions(gvio_isolate PRIVATE GVIO_TOOL=1)
endif()
