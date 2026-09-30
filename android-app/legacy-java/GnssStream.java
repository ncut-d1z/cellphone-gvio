package com.gvio.app;

import android.content.Context;
import android.location.GnssMeasurementsEvent;
import android.location.GnssStatus;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;

/**
 * GNSS 采集：GPS_PROVIDER 位置(单调钟时间戳) + GnssMeasurements 卫星质量信息。
 * 原始伪距/载波相位量测的融合为后续扩展，v1 采集其星座/载噪比用于可用性判断。
 */
public final class GnssStream {

    private final LocationManager lm;
    private HandlerThread thread;
    private Handler handler;
    private boolean running = false;
    private long handle;
    private final StringBuilder info = new StringBuilder();

    public GnssStream(Context ctx) {
        lm = (LocationManager) ctx.getSystemService(Context.LOCATION_SERVICE);
    }

    public void start(long nativeHandle) {
        this.handle = nativeHandle;
        thread = new HandlerThread("gvio-gnss");
        thread.start();
        handler = new Handler(thread.getLooper());

        try {
            if (lm.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                lm.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 0f,
                        locationListener, handler.getLooper());
                info.append("GPS_PROVIDER requested\n");
            } else {
                info.append("GPS_PROVIDER disabled\n");
            }
        } catch (SecurityException e) {
            info.append("no location permission: ").append(e.getMessage()).append("\n");
        }

        try {
            lm.registerGnssMeasurementsCallback(measurementsCallback, handler);
            info.append("GnssMeasurements registered\n");
        } catch (Exception e) {
            info.append("GnssMeasurements unavailable: ").append(e.getMessage()).append("\n");
        }

        try {
            lm.registerGnssStatusCallback(statusCallback, handler);
        } catch (Exception ignored) {
        }

        running = true;
    }

    private final LocationListener locationListener = new LocationListener() {
        @Override
        public void onLocationChanged(Location l) {
            if (!running || l == null) return;
            long t = l.getElapsedRealtimeNanos();
            float acc = l.hasAccuracy() ? l.getAccuracy() : 50f;
            float speed = l.hasSpeed() ? l.getSpeed() : 0f;
            float bearing = l.hasBearing() ? l.getBearing() : 0f;
            NativeFusion.nativeFeedGpsFix(handle, t, l.getLatitude(), l.getLongitude(),
                    l.getAltitude(), acc, speed, bearing);
        }

        @Override
        public void onStatusChanged(String provider, int status, Bundle extras) {
        }

        @Override
        public void onProviderEnabled(String provider) {
        }

        @Override
        public void onProviderDisabled(String provider) {
        }
    };

    private final GnssMeasurementsEvent.Callback measurementsCallback =
            new GnssMeasurementsEvent.Callback() {
                @Override
                public void onGnssMeasurementsReceived(GnssMeasurementsEvent e) {
                    if (!running || e == null || e.getMeasurements() == null) return;
                    GnssMeasurementsEvent.Measurement[] ms =
                            e.getMeasurements().toArray(new GnssMeasurementsEvent.Measurement[0]);
                    int n = ms.length;
                    if (n == 0) return;
                    int[] constellations = new int[n];
                    float[] cn0 = new float[n];
                    boolean anyDualFreq = false;
                    for (int i = 0; i < n; i++) {
                        constellations[i] = ms[i].getConstellationType();
                        cn0[i] = (float) ms[i].getCn0DbHz();
                        double freq = ms[i].getCarrierFrequencyHz();
                        if (Math.abs(freq - 1176.45e6) < 1e4 || Math.abs(freq - 1176.45e6) < 1e4) {
                            anyDualFreq = true;
                        }
                    }
                    long t;
                    try {
                        t = e.getClock().getElapsedRealtimeNanos();
                    } catch (Exception ex) {
                        t = SystemClock.elapsedRealtimeNanos();
                    }
                    NativeFusion.nativeFeedGnssSats(handle, t, constellations, cn0);
                }
            };

    private final GnssStatus.Callback statusCallback = new GnssStatus.Callback() {
        @Override
        public void onSatelliteStatusChanged(GnssStatus s) {
            if (!running) return;
            info.setLength(0);
            info.append("sats=").append(s.getSatelliteCount());
        }
    };

    public String getInfo() {
        return info.toString();
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        running = false;
        try {
            lm.removeUpdates(locationListener);
            lm.unregisterGnssMeasurementsCallback(measurementsCallback);
            lm.unregisterGnssStatusCallback(statusCallback);
        } catch (Exception ignored) {
        }
        if (thread != null) thread.quitSafely();
        thread = null;
    }
}
