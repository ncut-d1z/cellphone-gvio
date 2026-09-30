package com.gvio.app;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Size;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Camera2 采集：主摄 YUV_420_888，优先 1080p@30fps，降级到可用最大 16:9。
 * Y 平面拷贝进持久 direct ByteBuffer 后经 JNI 送入后端，帧时间戳(相机 SENSOR 域)
 * 同步交给 ClockSync 用于时钟对齐。
 */
public final class CameraStream {

    public static final class FrameMeta {
        public volatile long sensorTimestampNs = 0;
        public volatile long exposureNs = 0;
        public volatile float iso = 0;
    }

    private final CameraManager cameraManager;
    private final Context ctx;
    private String cameraId = "0";
    private Size size;
    private int fps = 30;
    private CameraDevice device;
    private CameraCaptureSession session;
    private ImageReader reader;
    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private long handle;
    private ClockSync clockSync;
    private volatile boolean running = false;
    private final AtomicReference<FrameMeta> latestMeta = new AtomicReference<>(new FrameMeta());
    private final AtomicLong frameCount = new AtomicLong(0);
    private ByteBuffer yBuffer;

    public final StringBuilder info = new StringBuilder();

    public CameraStream(Context ctx) {
        this.ctx = ctx;
        cameraManager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
    }

    public Size getSize() {
        return size;
    }

    public int getFps() {
        return fps;
    }

    public FrameMeta meta() {
        return latestMeta.get();
    }

    public String getTimestampSource() {
        return info.toString();
    }

    public boolean isRunning() {
        return running;
    }

    private static String probeTimestampSource(CameraCharacteristics cc) {
        Integer ts = cc.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
        if (ts != null && ts == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) {
            return "realtime";
        }
        return "unknown";
    }

    public void start(long nativeHandle, ClockSync sync) throws CameraAccessException {
        this.handle = nativeHandle;
        this.clockSync = sync;

        cameraThread = new HandlerThread("gvio-camera");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());

        cameraId = null;
        for (String id : cameraManager.getCameraIdList()) {
            CameraCharacteristics cc = cameraManager.getCameraCharacteristics(id);
            Integer facing = cc.get(CameraCharacteristics.LENS_FACING);
            if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                cameraId = id;
                break;
            }
        }
        if (cameraId == null) cameraId = cameraManager.getCameraIdList()[0];

        CameraCharacteristics cc = cameraManager.getCameraCharacteristics(cameraId);
        info.append("id=").append(cameraId).append(" timestampSource=")
                .append(probeTimestampSource(cc)).append("\n");

        StreamConfigurationMap map =
                cc.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        Size[] sizes = map.getOutputSizes(ImageFormat.YUV_420_888);
        Size best = null;
        for (Size s : sizes) {
            if (s.getHeight() > s.getWidth()) continue;
            double ratio = (double) s.getWidth() / s.getHeight();
            if (Math.abs(ratio - 16.0 / 9.0) > 0.05) continue;
            if (s.getHeight() > 1080) continue;
            if (best == null || s.getWidth() > best.getWidth()) best = s;
        }
        if (best == null) best = sizes[0];
        size = best;

        android.util.Range<Integer>[] fpsRanges =
                cc.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        fps = 30;
        if (fpsRanges != null) {
            for (android.util.Range<Integer> r : fpsRanges) {
                if (r.getLower() <= 30 && r.getUpper() >= 30) {
                    fps = Math.min(30, r.getUpper());
                    break;
                }
            }
        }
        info.append("size=").append(size).append(" fps=").append(fps);

        reader = ImageReader.newInstance(size.getWidth(), size.getHeight(),
                ImageFormat.YUV_420_888, 8);
        reader.setOnImageAvailableListener(onImage, cameraHandler);

        cameraManager.openCamera(cameraId, stateCallback, cameraHandler);
    }

    private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice d) {
            device = d;
            try {
                createSession();
            } catch (CameraAccessException e) {
                info.append("\nERR session: ").append(e.getMessage());
            }
        }

        @Override
        public void onDisconnected(CameraDevice d) {
            d.close();
            device = null;
        }

        @Override
        public void onError(CameraDevice d, int error) {
            d.close();
            device = null;
            info.append("\nERR camera: ").append(error);
        }
    };

    private void createSession() throws CameraAccessException {
        final CaptureRequest.Builder req =
                device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
        Surface surf = reader.getSurface();
        req.addTarget(surf);
        req.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        req.set(CaptureRequest.CONTROL_AF_MODE,
                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO);
        req.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
                new android.util.Range<>(Math.min(fps, 30), fps));

        device.createCaptureSession(Collections.singletonList(surf), new CameraCaptureSession.StateCallback() {
            @Override
            public void onConfigured(CameraCaptureSession s) {
                session = s;
                try {
                    s.setRepeatingRequest(req.build(), captureCallback, cameraHandler);
                    running = true;
                } catch (CameraAccessException e) {
                    info.append("\nERR start req: ").append(e.getMessage());
                }
            }

            @Override
            public void onConfigureFailed(CameraCaptureSession s) {
                info.append("\nERR configure failed");
            }
        }, cameraHandler);
    }

    private final CameraCaptureSession.CaptureCallback captureCallback =
            new CameraCaptureSession.CaptureCallback() {
                @Override
                public void onCaptureCompleted(CameraCaptureSession s, CaptureRequest r,
                                               CaptureResult result) {
                    FrameMeta m = latestMeta.getAndSet(new FrameMeta());
                    Long ts = result.get(CaptureResult.SENSOR_TIMESTAMP);
                    Long exp = result.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                    Float iso = result.get(CaptureResult.SENSOR_SENSITIVITY);
                    if (ts != null) m.sensorTimestampNs = ts;
                    if (exp != null) m.exposureNs = exp;
                    if (iso != null) m.iso = iso;
                }
            };

    private final ImageReader.OnImageAvailableListener onImage =
            new ImageReader.OnImageAvailableListener() {
                @Override
                public void onImageAvailable(ImageReader r) {
                    if (!running) return;
                    Image image = null;
                    try {
                        image = r.acquireLatestImage();
                        if (image == null) return;
                        long camTs = image.getTimestamp();
                        clockSync.observe(camTs);

                        Image.Plane y = image.getPlanes()[0];
                        ByteBuffer buf = y.getBuffer();
                        int yStride = y.getRowStride();
                        int height = image.getHeight();
                        int needed = yStride * height;
                        if (yBuffer == null || yBuffer.capacity() < needed) {
                            yBuffer = ByteBuffer.allocateDirect(needed);
                        }
                        yBuffer.clear();
                        if (buf.hasArray()) {
                            yBuffer.put(buf.array(), 0, needed);
                        } else {
                            ByteBuffer dup = buf.duplicate();
                            dup.limit(needed);
                            yBuffer.put(dup);
                        }
                        yBuffer.position(0);

                        FrameMeta m = latestMeta.get();
                        long exp = m.exposureNs;
                        float iso = m.iso;
                        frameCount.incrementAndGet();
                        NativeFusion.nativeFeedImage(handle, camTs, exp, iso,
                                image.getWidth(), height, yStride, yBuffer);
                    } catch (Exception e) {
                        info.append("\nERR frame: ").append(e.getMessage());
                    } finally {
                        if (image != null) image.close();
                    }
                }
            };

    public long getFrameCount() {
        return frameCount.get();
    }

    public void stop() {
        running = false;
        try {
            if (session != null) session.close();
            if (device != null) device.close();
            if (reader != null) reader.close();
        } catch (Exception ignored) {
        }
        session = null;
        device = null;
        reader = null;
        if (cameraThread != null) cameraThread.quitSafely();
        cameraThread = null;
    }
}
