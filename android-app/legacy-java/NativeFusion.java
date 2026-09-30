package com.gvio.app;

import java.nio.ByteBuffer;

/**
 * JNI 桥接：调用 C++ 后端(backend/src/jni/gvio_jni.cpp)。
 * 所有时间戳均为纳秒；IMU/磁强计为 CLOCK_MONOTONIC 域，图像为相机 SENSOR 域
 * (由 nativeSetClockOffset 转换为单调钟域)，GNSS 位置用 Location.getElapsedRealtimeNanos()。
 */
public final class NativeFusion {

    static {
        System.loadLibrary("gvio_fusion");
    }

    private NativeFusion() {
    }

    /** 创建融合后端实例，configJson 见 Config.java；返回 native handle。 */
    public static native long nativeCreate(String configJson);

    public static native void nativeDestroy(long handle);

    /** 启动融合(接收数据前必须调用)。 */
    public static native void nativeStart(long handle);

    public static native void nativeStop(long handle);

    public static native void nativeReset(long handle);

    /** 进入磁强计硬磁校准模式(采集 N 个样本后自动估计零偏并退出)。 */
    public static native void nativeCalibrateMag(long handle);

    /** 设置 相机SENSOR时钟 -> CLOCK_MONOTONIC 的偏移量(纳秒)。 */
    public static native void nativeSetClockOffset(long handle, double offsetNs);

    /** values = [ax,ay,az,gx,gy,gz]，biases = [bx,by,bz,bgx,bgy,bgz] (UNCALIBRATED 六元组)。 */
    public static native void nativeFeedImu(long handle, long tNs, float[] values, float[] biases);

    /** values = [mx,my,mz]，biases = [bx,by,bz] (UNCALIBRATED)。 */
    public static native void nativeFeedMag(long handle, long tNs, float[] values, float[] biases);

    /** GPS 位置量测；tNs 取 Location.getElapsedRealtimeNanos()。 */
    public static native void nativeFeedGpsFix(long handle, long tNs, double lat, double lon,
                                               double alt, float accH, float speed, float bearing);

    /** GNSS 卫星质量信息(星座类型 + 载噪比)，用于量测可用性判断。 */
    public static native void nativeFeedGnssSats(long handle, long tNs,
                                                 int[] constellations, float[] cn0);

    /** 图像帧(Y 平面)。tNs 为相机 SENSOR 时钟域；yPlane 必须为 direct ByteBuffer。 */
    public static native void nativeFeedImage(long handle, long tNs, long exposureNs, float iso,
                                              int width, int height, int yStride,
                                              ByteBuffer yPlane);

    /** 状态快照 JSON(供 UI 与调试)。 */
    public static native String nativeGetStatusJson(long handle);
}
