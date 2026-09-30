package com.gvio.app;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.SystemClock;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;

/**
 * 前台采集服务：持有 CameraStream / SensorStream / GnssStream 与 C++ 后端句柄，
 * 负责启动/停止采集并把状态暴露给 UI。
 */
public final class AcquisitionService extends Service {

    public static final String ACTION_START = "com.gvio.app.START";
    public static final String ACTION_STOP = "com.gvio.app.STOP";
    public static final String ACTION_MAG_CAL = "com.gvio.app.MAG_CAL";
    public static final String ACTION_RESET = "com.gvio.app.RESET";

    private static final String CHANNEL_ID = "gvio";
    private static final int NOTIF_ID = 1;

    public static volatile boolean running = false;
    public static volatile String lastStatusJson = "";

    private PowerManager.WakeLock wakeLock;
    private Handler ticker;
    private long handle = 0;
    private ClockSync clockSync;
    private SensorStream sensorStream;
    private CameraStream cameraStream;
    private GnssStream gnssStream;
    private boolean clockOffsetPushed = false;

    @Override
    public void onCreate() {
        super.onCreate();
        createChannel();
        startForeground(NOTIF_ID, buildNotification("初始化中..."));
        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "gvio:acq");
        wakeLock.acquire();
        ticker = new Handler();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent == null) return START_NOT_STICKY;
        String action = intent.getAction();
        if (ACTION_START.equals(action)) {
            init();
        } else if (ACTION_STOP.equals(action)) {
            shutdown();
        } else if (ACTION_MAG_CAL.equals(action) && handle != 0) {
            NativeFusion.nativeCalibrateMag(handle);
        } else if (ACTION_RESET.equals(action) && handle != 0) {
            NativeFusion.nativeReset(handle);
        }
        return START_NOT_STICKY;
    }

    private void init() {
        if (running) return;
        try {
            copyCerts();

            String config = Config.build(this, 1920, 1080, 30);
            handle = NativeFusion.nativeCreate(config);
            if (handle == 0) {
                lastStatusJson = "{\"error\":\"nativeCreate failed\"}";
                return;
            }
            clockSync = new ClockSync(readTimestampSource(config));

            sensorStream = new SensorStream(this);
            gnssStream = new GnssStream(this);
            cameraStream = new CameraStream(this);

            sensorStream.start(handle);
            gnssStream.start(handle);
            cameraStream.start(handle, clockSync);

            NativeFusion.nativeStart(handle);
            running = true;
            startTicker();
        } catch (Exception e) {
            lastStatusJson = "{\"error\":\"" + e + "\"}";
            shutdown();
        }
    }

    private static String readTimestampSource(String configJson) {
        try {
            org.json.JSONObject o = new org.json.JSONObject(configJson);
            return o.getJSONObject("camera").optString("sensorTimestampSource", "unknown");
        } catch (Exception e) {
            return "unknown";
        }
    }

    private void copyCerts() {
        File dir = new File(getFilesDir(), "certs");
        if (dir.exists()) return;
        dir.mkdirs();
        for (String asset : new String[]{"server.crt", "server.key"}) {
            try (InputStream in = getAssets().open("certs/" + asset);
                 FileOutputStream out = new FileOutputStream(new File(dir, asset))) {
                byte[] buf = new byte[8192];
                int n;
                while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            } catch (Exception ignored) {
            }
        }
    }

    private void startTicker() {
        ticker.postDelayed(tick, 500);
    }

    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            if (!running) return;
            if (!clockOffsetPushed && clockSync != null && clockSync.isReady()) {
                NativeFusion.nativeSetClockOffset(handle, clockSync.getOffsetNs());
                clockOffsetPushed = true;
            }
            StringBuilder sb = new StringBuilder();
            try {
                String nativeJson = NativeFusion.nativeGetStatusJson(handle);
                sb.append(nativeJson);
                sb.append(", \"acq\":{");
                sb.append("\"clock\":\"").append(clockSync == null ? "?" : clockSync.describe()).append("\",");
                sb.append("\"frames\":").append(cameraStream == null ? 0 : cameraStream.getFrameCount()).append(",");
                sb.append("\"camera\":\"").append(cameraStream == null ? "?" :
                        cameraStream.getTimestampSource().replace("\n", "|")).append("\",");
                sb.append("\"sensors\":\"").append(sensorStream == null ? "?" :
                        sensorStream.getInfo().replace("\n", "|")).append("\",");
                sb.append("\"gnss\":\"").append(gnssStream == null ? "?" :
                        gnssStream.getInfo().replace("\n", "|")).append("\"");
                sb.append("}");
            } catch (Throwable t) {
                sb.append("{\"error\":\"").append(t).append("\"}");
            }
            lastStatusJson = sb.toString();
            ticker.postDelayed(this, 500);
        }
    };

    private void shutdown() {
        running = false;
        if (ticker != null) ticker.removeCallbacksAndMessages(null);
        try {
            if (handle != 0) NativeFusion.nativeStop(handle);
            if (sensorStream != null) sensorStream.stop();
            if (gnssStream != null) gnssStream.stop();
            if (cameraStream != null) cameraStream.stop();
            if (handle != 0) {
                NativeFusion.nativeDestroy(handle);
                handle = 0;
            }
        } catch (Throwable ignored) {
        }
        sensorStream = null;
        gnssStream = null;
        cameraStream = null;
        clockSync = null;
        stopForeground(true);
        stopSelf();
    }

    private void createChannel() {
        NotificationManager nm = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        NotificationChannel ch = new NotificationChannel(CHANNEL_ID, "GVIO Fusion",
                NotificationManager.IMPORTANCE_LOW);
        nm.createNotificationChannel(ch);
    }

    private Notification buildNotification(String text) {
        Intent i = new Intent(this, MainActivity.class);
        PendingIntent pi = PendingIntent.getActivity(this, 0, i,
                PendingIntent.FLAG_UPDATE_CURRENT);
        return new Notification.Builder(this, CHANNEL_ID)
                .setContentTitle("GVIO Fusion 采集运行中")
                .setContentText(text)
                .setContentIntent(pi)
                .setOngoing(true)
                .build();
    }

    @Override
    public void onDestroy() {
        shutdown();
        if (wakeLock != null && wakeLock.isHeld()) wakeLock.release();
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }
}
