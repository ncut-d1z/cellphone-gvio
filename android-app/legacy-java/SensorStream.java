package com.gvio.app;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * 采集 IMU 与磁强计原始数据(UNCALIBRATED 变体)，直接经 JNI 送入后端。
 * 事件时间戳为 CLOCK_MONOTONIC 纳秒域。
 */
public final class SensorStream implements SensorEventListener {

    private final SensorManager sm;
    private Sensor accel, gyro, mag;
    private boolean running = false;
    private long handle;
    private final StringBuilder info = new StringBuilder();

    private final float[] vImu = new float[6];   // [ax,ay,az,gx,gy,gz]
    private final float[] bImu = new float[6];   // [bx,by,bz,bgx,bgy,bgz]
    private final float[] vMag = new float[3];
    private final float[] bMag = new float[3];

    private long pendingAccelT = -1;
    private final float[] pendingAccel = new float[3];
    private final float[] pendingAccelB = new float[3];

    public SensorStream(Context ctx) {
        sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        mag = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED);
        if (accel == null) accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (gyro == null) gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if (mag == null) mag = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        info.append("ACC:").append(name(accel)).append(" ").append(maxRate(accel)).append("Hz\n");
        info.append("GYRO:").append(name(gyro)).append(" ").append(maxRate(gyro)).append("Hz\n");
        info.append("MAG:").append(name(mag)).append(" ").append(maxRate(mag)).append("Hz");
    }

    private static String name(Sensor s) {
        if (s == null) return "N/A";
        return s.getName() + "/" + s.getVendor() + " range=" + s.getMaximumRange();
    }

    private static int maxRate(Sensor s) {
        if (s == null || s.getMinDelay() <= 0) return 0;
        return (int) (1e6f / s.getMinDelay());
    }

    public void start(long nativeHandle) {
        this.handle = nativeHandle;
        int d = SensorManager.SENSOR_DELAY_FASTEST;
        if (accel != null) sm.registerListener(this, accel, d);
        if (gyro != null) sm.registerListener(this, gyro, d);
        if (mag != null) sm.registerListener(this, mag, d);
        running = true;
    }

    public void stop() {
        running = false;
        sm.unregisterListener(this);
    }

    public boolean isRunning() {
        return running;
    }

    public String getInfo() {
        return info.toString();
    }

    @Override
    public void onSensorChanged(SensorEvent e) {
        if (!running) return;
        long t = e.timestamp;
        int type = e.sensor.getType();
        if (type == Sensor.TYPE_ACCELEROMETER || type == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) {
            float[] v = e.values;
            if (v.length >= 6) {
                System.arraycopy(v, 0, vImu, 0, 3);
                System.arraycopy(v, 3, bImu, 0, 3);
            } else {
                System.arraycopy(v, 0, vImu, 0, 3);
                bImu[0] = bImu[1] = bImu[2] = 0;
            }
            pendingAccelT = t;
            System.arraycopy(vImu, 0, pendingAccel, 0, 3);
            System.arraycopy(bImu, 0, pendingAccelB, 0, 3);
            return;
        }
        if (type == Sensor.TYPE_GYROSCOPE || type == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
            float[] v = e.values;
            if (v.length >= 6) {
                System.arraycopy(v, 0, vImu, 3, 3);
                System.arraycopy(v, 3, bImu, 3, 3);
            } else {
                System.arraycopy(v, 0, vImu, 3, 3);
                bImu[3] = bImu[4] = bImu[5] = 0;
            }
            if (pendingAccelT == t) {
                System.arraycopy(pendingAccel, 0, vImu, 0, 3);
                System.arraycopy(pendingAccelB, 0, bImu, 0, 3);
                pendingAccelT = -1;
            }
            NativeFusion.nativeFeedImu(handle, t, vImu, bImu);
            return;
        }
        if (type == Sensor.TYPE_MAGNETIC_FIELD || type == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
            float[] v = e.values;
            if (v.length >= 6) {
                System.arraycopy(v, 0, vMag, 0, 3);
                System.arraycopy(v, 3, bMag, 0, 3);
            } else {
                System.arraycopy(v, 0, vMag, 0, 3);
                bMag[0] = bMag[1] = bMag[2] = 0;
            }
            NativeFusion.nativeFeedMag(handle, t, vMag, bMag);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
