// JNI 桥接层: 对应 android-app/.../NativeFusion.java
// 库名 gvio_fusion(CMake 中 OUTPUT_NAME gvio_fusion)
#include "gvio/config.h"
#include "gvio/fusion.h"

#include <jni.h>

#include <cstdint>
#include <cstring>
#include <string>

namespace {

using gvio::Config;
using gvio::FusionEngine;

FusionEngine* fromHandle(jlong h) {
    return reinterpret_cast<FusionEngine*>(static_cast<uintptr_t>(h));
}

std::string jstr(JNIEnv* env, jstring s) {
    const char* c = env->GetStringUTFChars(s, nullptr);
    std::string out = c ? c : "";
    if (c) env->ReleaseStringUTFChars(s, c);
    return out;
}

}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_gvio_app_NativeFusion_nativeCreate(JNIEnv* env, jclass, jstring configJson) {
    std::string jsonStr = jstr(env, configJson);
    Config cfg = Config::fromJson(jsonStr);
    if (!cfg.valid) return 0;
    return reinterpret_cast<jlong>(new FusionEngine(cfg));
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeDestroy(JNIEnv*, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (e) {
        e->stop();
        delete e;
    }
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeStart(JNIEnv*, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->start();
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeStop(JNIEnv*, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->stop();
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeReset(JNIEnv*, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->reset();
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeCalibrateMag(JNIEnv*, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->startMagCal();
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeSetClockOffset(JNIEnv*, jclass, jlong handle, jdouble offsetNs) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->setClockOffset(offsetNs);
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeFeedImu(JNIEnv* env, jclass, jlong handle, jlong tNs,
                                             jfloatArray values, jfloatArray biases) {
    FusionEngine* e = fromHandle(handle);
    if (!e) return;
    float v[6], b[6];
    env->GetFloatArrayRegion(values, 0, 6, v);
    env->GetFloatArrayRegion(biases, 0, 6, b);
    e->feedImu(tNs, v, b);
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeFeedMag(JNIEnv* env, jclass, jlong handle, jlong tNs,
                                             jfloatArray values, jfloatArray biases) {
    FusionEngine* e = fromHandle(handle);
    if (!e) return;
    float v[3], b[3];
    env->GetFloatArrayRegion(values, 0, 3, v);
    env->GetFloatArrayRegion(biases, 0, 3, b);
    e->feedMag(tNs, v, b);
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeFeedGpsFix(JNIEnv*, jclass, jlong handle, jlong tNs,
                                                jdouble lat, jdouble lon, jdouble alt,
                                                jfloat accH, jfloat speed, jfloat bearing) {
    FusionEngine* e = fromHandle(handle);
    if (e) e->feedGpsFix(tNs, lat, lon, alt, accH, speed, bearing);
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeFeedGnssSats(JNIEnv* env, jclass, jlong handle, jlong tNs,
                                                  jintArray constellations, jfloatArray cn0) {
    FusionEngine* e = fromHandle(handle);
    if (!e) return;
    jsize n = env->GetArrayLength(constellations);
    if (n <= 0) return;
    std::vector<int> cons(n);
    std::vector<float> cn(n);
    env->GetIntArrayRegion(constellations, 0, n, cons.data());
    env->GetFloatArrayRegion(cn0, 0, n, cn.data());
    e->feedGnssSats(tNs, cons.data(), cn.data(), static_cast<int>(n));
}

JNIEXPORT void JNICALL
Java_com_gvio_app_NativeFusion_nativeFeedImage(JNIEnv* env, jclass, jlong handle, jlong tNs,
                                               jlong exposureNs, jfloat iso, jint width,
                                               jint height, jint yStride, jobject yPlane) {
    FusionEngine* e = fromHandle(handle);
    if (!e) return;
    void* ptr = env->GetDirectBufferAddress(yPlane);
    if (!ptr) {
        jclass exc = env->FindClass("java/lang/IllegalArgumentException");
        if (exc) env->ThrowNew(exc, "yPlane must be a direct ByteBuffer");
        return;
    }
    jlong cap = env->GetDirectBufferCapacity(yPlane);
    if (cap < 0) cap = 0;
    e->feedImage(tNs, exposureNs, iso, width, height, yStride,
                 static_cast<const uint8_t*>(ptr), static_cast<size_t>(cap));
}

JNIEXPORT jstring JNICALL
Java_com_gvio_app_NativeFusion_nativeGetStatusJson(JNIEnv* env, jclass, jlong handle) {
    FusionEngine* e = fromHandle(handle);
    if (!e) return env->NewStringUTF("{\"error\":\"no engine\"}");
    std::string s = e->statusJson();
    return env->NewStringUTF(s.c_str());
}

}  // extern "C"
