package com.gvio.app;

import android.content.Context;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;

/**
 * 构建传给 nativeCreate 的配置 JSON：相机内参(取自 CameraCharacteristics)、
 * IMU 噪声默认值、磁力计参数、服务器端口、证书路径等。
 */
public final class Config {

    private Config() {
    }

    public static String build(Context ctx, int imgW, int imgH, int fps) {
        try {
            JSONObject c = new JSONObject();

            JSONObject cam = new JSONObject();
            cam.put("width", imgW);
            cam.put("height", imgH);
            cam.put("fps", fps);
            cam.put("timestampSource", "sensor");

            CameraManager mgr = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
            String camId = null;
            try {
                for (String id : mgr.getCameraIdList()) {
                    CameraCharacteristics cc = mgr.getCameraCharacteristics(id);
                    Integer facing = cc.get(CameraCharacteristics.LENS_FACING);
                    if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                        camId = id;
                        break;
                    }
                }
            } catch (Exception ignored) {
            }
            if (camId == null && mgr.getCameraIdList().length > 0) {
                camId = mgr.getCameraIdList()[0];
            }
            cam.put("cameraId", camId != null ? camId : "0");

            if (camId != null) {
                CameraCharacteristics cc = mgr.getCameraCharacteristics(camId);
                float[] K = cc.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
                if (K != null && K.length >= 4) {
                    JSONArray k = new JSONArray();
                    for (float v : K) k.put(v);
                    cam.put("intrinsics", k);
                }
                float[] dist = cc.get(CameraCharacteristics.LENS_DISTORTION);
                if (dist != null) {
                    JSONArray d = new JSONArray();
                    for (float v : dist) d.put(v);
                    cam.put("distortion", d);
                }
                Integer ts = cc.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
                cam.put("sensorTimestampSource",
                        ts != null && ts == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
                                ? "realtime" : "unknown");
            }
            c.put("camera", cam);

            JSONObject imu = new JSONObject();
            imu.put("gyroNoise", 3.0e-4);      // rad/s/√Hz，默认值，可后续实测标定
            imu.put("accelNoise", 2.0e-2);     // m/s²/√Hz
            imu.put("gyroBiasWalk", 1.0e-6);   // rad/s²/√Hz
            imu.put("accelBiasWalk", 1.0e-4);  // m/s³/√Hz
            c.put("imu", imu);

            JSONObject mag = new JSONObject();
            mag.put("declinationDeg", 0.0);    // 磁偏角，真机按地区填写(度，东偏为正)
            mag.put("inclinationDeg", 0.0);    // 磁倾角
            mag.put("fieldUT", 50.0);          // 地磁强度近似(µT)
            mag.put("yawNoiseRad", 0.087);     // 5°
            mag.put("enabled", true);
            c.put("mag", mag);

            JSONObject gps = new JSONObject();
            gps.put("enabled", true);
            gps.put("minSats", 4);
            c.put("gps", gps);

            JSONObject filter = new JSONObject();
            filter.put("cameraWindowSize", 20);
            filter.put("featureNoisePx", 1.5);
            filter.put("maxFeatures", 300);
            filter.put("minParallaxDeg", 1.0);
            filter.put("minTriObs", 3);
            c.put("filter", filter);

            JSONObject server = new JSONObject();
            server.put("host", "127.0.0.1");
            server.put("port", 8443);
            File dir = new File(ctx.getFilesDir(), "certs");
            server.put("certPath", new File(dir, "server.crt").getAbsolutePath());
            server.put("keyPath", new File(dir, "server.key").getAbsolutePath());
            c.put("server", server);

            return c.toString();
        } catch (Exception e) {
            return "{}";
        }
    }
}
