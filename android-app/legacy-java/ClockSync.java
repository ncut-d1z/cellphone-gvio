package com.gvio.app;

import android.os.SystemClock;

/**
 * 相机 SENSOR 时钟域 -> CLOCK_MONOTONIC 的偏移估计。
 * - 若 HAL 声明 REALTIME：偏移 = wallClock - monoClock(一次采样即可，两者均为稳定时钟)。
 * - 若 UNKNOWN：对前 N 帧的 (帧SENSOR时间戳, 回调时单调钟) 求中位偏移
 *   (回调抖动 ~1 帧周期，30fps 下约 ±16ms，v1 可接受，后续可用 GNSS/IMU 互相关精化)。
 */
public final class ClockSync {

    private static final int MAX_PAIRS = 60;

    private long[] camTs = new long[MAX_PAIRS];
    private long[] monoTs = new long[MAX_PAIRS];
    private int count = 0;
    private boolean realtimeMode = false;
    private long realtimeOffsetNs = 0;
    private boolean ready = false;
    private double estimatedOffsetNs = 0;

    public ClockSync(String sensorTimestampSource) {
        realtimeMode = "realtime".equals(sensorTimestampSource);
        if (realtimeMode) {
            long wall = System.currentTimeMillis() * 1000000L;
            long mono = SystemClock.elapsedRealtimeNanos();
            realtimeOffsetNs = mono - wall;
            ready = true;
            estimatedOffsetNs = realtimeOffsetNs;
        }
    }

    /** 在 onImageAvailable 处理该帧时调用。camNs 为 Image.getTimestamp()。 */
    public synchronized void observe(long camNs) {
        if (realtimeMode) return;
        if (count < MAX_PAIRS) {
            camTs[count] = camNs;
            monoTs[count] = SystemClock.elapsedRealtimeNanos();
            count++;
            if (count == MAX_PAIRS) {
                double[] diffs = new double[MAX_PAIRS];
                for (int i = 0; i < MAX_PAIRS; i++) diffs[i] = monoTs[i] - camTs[i];
                java.util.Arrays.sort(diffs);
                estimatedOffsetNs = diffs[MAX_PAIRS / 2];
                ready = true;
            }
        }
    }

    public synchronized boolean isReady() {
        return ready;
    }

    public synchronized double getOffsetNs() {
        return estimatedOffsetNs;
    }

    /** 将相机域时间戳转换为单调钟域。 */
    public synchronized long toMonotonic(long camNs) {
        return camNs + (long) estimatedOffsetNs;
    }

    public String describe() {
        return (realtimeMode ? "realtime" : "regression")
                + " offset=" + (long) estimatedOffsetNs + "ns ready=" + ready;
    }
}
