package com.gvio.app;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.location.GnssStatus;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ViewFlipper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public class MainActivity extends Activity implements SensorEventListener {

    private static final String TAG = "gvio";
    private static final int REQ_PERM = 1;

    private RadioGroup tabs;
    private ViewFlipper content;
    private TextureView camViewBack, camViewFront;
    private TextView camInfoBack, camInfoFront;
    private RadioGroup ctlTarget;
    private SeekBar zoomSeek, focusSeek, evSeek;
    private TextView zoomLabel, focusLabel, evLabel;
    private CheckBox focusCb;
    private TextView accelX, accelY, accelZ, accelHz;
    private TextView gyroX, gyroY, gyroZ, gyroHz, imuEvents;
    private TextView magInfo, gpsInfo;

    private SensorManager sm;
    private Sensor accel, gyro, mag;
    private volatile float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;
    private long aCount, gCount, mCount;
    private long aCountLast, gCountLast, mCountLast;

    private CameraManager cm;
    private CamCtl backCam, frontCam;

    private LocationManager lm;
    private volatile double lat, lon, alt, speed, bearing, accuracy;
    private volatile long fixT = -1;
    private volatile boolean hasFix;
    private volatile int satsUsed, satsTotal;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private final Runnable uiTick = new Runnable() {
        @Override
        public void run() {
            updateViews();
            ui.postDelayed(this, 100);
        }
    };

    // ---------------------------------------------------------------- camera
    private class CamCtl {
        final boolean front;
        final TextureView view;
        final TextView info;
        String camId;
        int orient;
        Size previewSize;
        boolean zoomOk, focusOk, evOk;
        float zoomMin = 1f, zoomMax = 1f, zoomRatio = 1f;
        float minFocusDist, focusDist;
        boolean manualFocus;
        int evMin, evMax, evSteps;
        CameraDevice device;
        CameraCaptureSession session;
        CaptureRequest.Builder builder;
        ImageReader fpsReader;
        long fpsFrames;
        boolean ready, opening;

        CamCtl(boolean f, TextureView v, TextView i) {
            front = f;
            view = v;
            info = i;
        }

        void open() {
            if (opening || cm == null || view == null) return;
            try {
                camId = null;
                String[] ids = cm.getCameraIdList();
                int want = front ? CameraCharacteristics.LENS_FACING_FRONT
                        : CameraCharacteristics.LENS_FACING_BACK;
                for (String id : ids) {
                    CameraCharacteristics ch = cm.getCameraCharacteristics(id);
                    Integer facing = ch.get(CameraCharacteristics.LENS_FACING);
                    if (facing != null && facing == want) {
                        camId = id;
                        break;
                    }
                }
                if (camId == null && !front && ids.length > 0) camId = ids[0];
                if (camId == null) {
                    info.setText(front ? "无前置相机" : "无后置相机");
                    return;
                }
                CameraCharacteristics ch = cm.getCameraCharacteristics(camId);
                Integer o = ch.get(CameraCharacteristics.SENSOR_ORIENTATION);
                orient = o != null ? o : 90;
                StreamConfigurationMap map =
                        ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                previewSize = pickPreviewSize(map);
                Range<Float> zr = ch.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE);
                zoomOk = zr != null && zr.getUpper() > zr.getLower();
                if (zoomOk) {
                    zoomMin = zr.getLower();
                    zoomMax = zr.getUpper();
                    zoomRatio = 1f;
                }
                Float md = ch.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
                focusOk = md != null && md > 0f;
                if (focusOk) {
                    minFocusDist = md;
                    focusDist = 0f;
                }
                Range<Integer> er = ch.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE);
                evOk = er != null;
                if (evOk) {
                    evMin = er.getLower();
                    evMax = er.getUpper();
                    evSteps = 0;
                }
                fpsReader = ImageReader.newInstance(64, 64, ImageFormat.YUV_420_888, 4);
                fpsReader.setOnImageAvailableListener(new ImageReader.OnImageAvailableListener() {
                    @Override
                    public void onImageAvailable(ImageReader r) {
                        android.media.Image img = r.acquireLatestImage();
                        if (img != null) img.close();
                        fpsFrames++;
                    }
                }, null);
                opening = true;
                cm.openCamera(camId, camState, ui);
            } catch (CameraAccessException | SecurityException e) {
                Log.e(TAG, "open camera failed " + (front ? "front" : "back"), e);
                info.setText("打开相机失败: " + e.getMessage());
                opening = false;
            }
        }

        private final CameraDevice.StateCallback camState = new CameraDevice.StateCallback() {
            @Override
            public void onOpened(CameraDevice dev) {
                device = dev;
                opening = false;
                createSession();
            }

            @Override
            public void onDisconnected(CameraDevice dev) {
                dev.close();
                device = null;
                ready = false;
                opening = false;
            }

            @Override
            public void onError(CameraDevice dev, int err) {
                Log.e(TAG, "camera error " + err);
                dev.close();
                device = null;
                ready = false;
                opening = false;
            }
        };

        void createSession() {
            if (device == null || !view.isAvailable() || previewSize == null) return;
            try {
                SurfaceTexture st = view.getSurfaceTexture();
                st.setDefaultBufferSize(previewSize.getWidth(), previewSize.getHeight());
                fitCenterCrop();
                Surface preview = new Surface(st);
                builder = device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                builder.addTarget(preview);
                builder.addTarget(fpsReader.getSurface());
                List<Surface> outs = new ArrayList<>();
                outs.add(preview);
                outs.add(fpsReader.getSurface());
                device.createCaptureSession(outs, new CameraCaptureSession.StateCallback() {
                    @Override
                    public void onConfigured(CameraCaptureSession s) {
                        session = s;
                        fitCenterCrop();
                        applyControls();
                        ready = true;
                        if (targetCam() == CamCtl.this) syncPanel();
                    }

                    @Override
                    public void onConfigureFailed(CameraCaptureSession s) {
                        Log.e(TAG, "session configure failed");
                    }
                }, ui);
            } catch (CameraAccessException | SecurityException e) {
                Log.e(TAG, "createSession failed", e);
            }
        }

        void fitCenterCrop() {
            if (view.getWidth() == 0 || view.getHeight() == 0 || previewSize == null) return;
            float bufW = previewSize.getWidth();
            float bufH = previewSize.getHeight();
            if (orient == 90 || orient == 270) {
                float t = bufW;
                bufW = bufH;
                bufH = t;
            }
            int vw = view.getWidth();
            int vh = view.getHeight();
            float scale = Math.max(vw / bufW, vh / bufH);
            float dx = (vw - bufW * scale) / 2f;
            float dy = (vh - bufH * scale) / 2f;
            Matrix m = new Matrix();
            m.setScale(scale, scale);
            m.postRotate(orient, bufW * scale / 2f + dx, bufH * scale / 2f + dy);
            m.postTranslate(dx, dy);
            view.setTransform(m);
        }

        void applyControls() {
            if (device == null || builder == null || session == null) return;
            try {
                if (zoomOk) {
                    builder.set(CaptureRequest.CONTROL_ZOOM_RATIO, zoomRatio);
                }
                if (focusOk) {
                    if (manualFocus) {
                        builder.set(CaptureRequest.CONTROL_AF_MODE,
                                CaptureRequest.CONTROL_AF_MODE_OFF);
                        builder.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDist);
                    } else {
                        builder.set(CaptureRequest.CONTROL_AF_MODE,
                                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                        builder.set(CaptureRequest.LENS_FOCUS_DISTANCE, 0f);
                    }
                }
                if (evOk) {
                    builder.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, evSteps);
                }
                session.setRepeatingRequest(builder.build(), null, ui);
            } catch (CameraAccessException e) {
                Log.e(TAG, "applyControls failed", e);
            }
        }

        void close() {
            ready = false;
            if (session != null) {
                session.close();
                session = null;
            }
            if (device != null) {
                device.close();
                device = null;
            }
            if (fpsReader != null) {
                fpsReader.close();
                fpsReader = null;
            }
        }
    }

    private final TextureView.SurfaceTextureListener texBack =
            new TextureView.SurfaceTextureListener() {
                @Override
                public void onSurfaceTextureAvailable(SurfaceTexture st, int w, int h) {
                    if (backCam == null) return;
                    if (backCam.previewSize != null) {
                        st.setDefaultBufferSize(backCam.previewSize.getWidth(),
                                backCam.previewSize.getHeight());
                        backCam.fitCenterCrop();
                    }
                    if (backCam.device == null && !backCam.opening) backCam.open();
                }

                @Override
                public void onSurfaceTextureSizeChanged(SurfaceTexture st, int w, int h) {
                    if (backCam != null) backCam.fitCenterCrop();
                }

                @Override
                public boolean onSurfaceTextureDestroyed(SurfaceTexture st) {
                    if (backCam != null) backCam.close();
                    return true;
                }

                @Override
                public void onSurfaceTextureUpdated(SurfaceTexture st) {
                }
            };

    private final TextureView.SurfaceTextureListener texFront =
            new TextureView.SurfaceTextureListener() {
                @Override
                public void onSurfaceTextureAvailable(SurfaceTexture st, int w, int h) {
                    if (frontCam == null) return;
                    if (frontCam.previewSize != null) {
                        st.setDefaultBufferSize(frontCam.previewSize.getWidth(),
                                frontCam.previewSize.getHeight());
                        frontCam.fitCenterCrop();
                    }
                    if (frontCam.device == null && !frontCam.opening) frontCam.open();
                }

                @Override
                public void onSurfaceTextureSizeChanged(SurfaceTexture st, int w, int h) {
                    if (frontCam != null) frontCam.fitCenterCrop();
                }

                @Override
                public boolean onSurfaceTextureDestroyed(SurfaceTexture st) {
                    if (frontCam != null) frontCam.close();
                    return true;
                }

                @Override
                public void onSurfaceTextureUpdated(SurfaceTexture st) {
                }
            };

    private final LocationListener locListener = new LocationListener() {
        @Override
        public void onLocationChanged(Location l) {
            lat = l.getLatitude();
            lon = l.getLongitude();
            alt = l.getAltitude();
            speed = l.getSpeed();
            bearing = l.getBearing();
            accuracy = l.getAccuracy();
            fixT = l.getTime();
            hasFix = true;
        }

        @Override
        public void onStatusChanged(String p, int s, android.os.Bundle b) {
        }

        @Override
        public void onProviderEnabled(String p) {
        }

        @Override
        public void onProviderDisabled(String p) {
            hasFix = false;
        }
    };

    private final GnssStatus.Callback gnssCb = new GnssStatus.Callback() {
        @Override
        public void onSatelliteStatusChanged(GnssStatus st) {
            satsTotal = st.getSatelliteCount();
            int used = 0;
            for (int i = 0; i < satsTotal; i++) {
                if (st.usedInFix(i)) used++;
            }
            satsUsed = used;
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        tabs = findViewById(R.id.tabs);
        content = findViewById(R.id.content);
        camViewBack = findViewById(R.id.camViewBack);
        camViewFront = findViewById(R.id.camViewFront);
        camInfoBack = findViewById(R.id.camInfoBack);
        camInfoFront = findViewById(R.id.camInfoFront);
        ctlTarget = findViewById(R.id.ctlTarget);
        zoomSeek = findViewById(R.id.zoomSeek);
        zoomLabel = findViewById(R.id.zoomLabel);
        focusCb = findViewById(R.id.focusCb);
        focusSeek = findViewById(R.id.focusSeek);
        focusLabel = findViewById(R.id.focusLabel);
        evSeek = findViewById(R.id.evSeek);
        evLabel = findViewById(R.id.evLabel);
        accelX = findViewById(R.id.accelX);
        accelY = findViewById(R.id.accelY);
        accelZ = findViewById(R.id.accelZ);
        accelHz = findViewById(R.id.accelHz);
        gyroX = findViewById(R.id.gyroX);
        gyroY = findViewById(R.id.gyroY);
        gyroZ = findViewById(R.id.gyroZ);
        gyroHz = findViewById(R.id.gyroHz);
        imuEvents = findViewById(R.id.imuEvents);
        magInfo = findViewById(R.id.magInfo);
        gpsInfo = findViewById(R.id.gpsInfo);

        tabs.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup g, int id) {
                if (id == R.id.tabImu) content.setDisplayedChild(1);
                else if (id == R.id.tabMag) content.setDisplayedChild(2);
                else if (id == R.id.tabGps) content.setDisplayedChild(3);
                else content.setDisplayedChild(0);
            }
        });

        ctlTarget.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup g, int id) {
                syncPanel();
            }
        });

        zoomSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar sb, int p, boolean fromUser) {
                CamCtl c = targetCam();
                if (c == null || !c.ready || !c.zoomOk) return;
                c.zoomRatio = c.zoomMin + (c.zoomMax - c.zoomMin) * p / 100f;
                zoomLabel.setText(String.format(Locale.US, "%.1fx", c.zoomRatio));
                if (fromUser) c.applyControls();
            }

            @Override
            public void onStartTrackingTouch(SeekBar sb) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar sb) {
                CamCtl c = targetCam();
                if (c != null && c.ready) c.applyControls();
            }
        });

        focusCb.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton b, boolean checked) {
                CamCtl c = targetCam();
                focusSeek.setEnabled(checked && c != null && c.ready && c.focusOk);
                if (c == null || !c.ready) return;
                c.manualFocus = checked;
                if (!checked) focusLabel.setText("∞");
                c.applyControls();
            }
        });

        focusSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar sb, int p, boolean fromUser) {
                CamCtl c = targetCam();
                if (c == null || !c.ready || !c.focusOk) return;
                c.focusDist = c.minFocusDist * (100 - p) / 100f;
                focusLabel.setText(focusLabelFor(c.focusDist));
                if (fromUser) c.applyControls();
            }

            @Override
            public void onStartTrackingTouch(SeekBar sb) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar sb) {
                CamCtl c = targetCam();
                if (c != null && c.ready) c.applyControls();
            }
        });

        evSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar sb, int p, boolean fromUser) {
                CamCtl c = targetCam();
                if (c == null || !c.ready || !c.evOk) return;
                c.evSteps = c.evMin + p;
                evLabel.setText(String.format(Locale.US, "%+d", c.evSteps));
                if (fromUser) c.applyControls();
            }

            @Override
            public void onStartTrackingTouch(SeekBar sb) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar sb) {
                CamCtl c = targetCam();
                if (c != null && c.ready) c.applyControls();
            }
        });

        sm = (SensorManager) getSystemService(SENSOR_SERVICE);
        cm = (CameraManager) getSystemService(CAMERA_SERVICE);
        lm = (LocationManager) getSystemService(LOCATION_SERVICE);
        accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mag = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        backCam = new CamCtl(false, camViewBack, camInfoBack);
        frontCam = new CamCtl(true, camViewFront, camInfoFront);
        camViewBack.setSurfaceTextureListener(texBack);
        camViewFront.setSurfaceTextureListener(texFront);

        if (checkSelfPermission(Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED
                || checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{
                    Manifest.permission.CAMERA,
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.BODY_SENSORS}, REQ_PERM);
        } else {
            startAll();
        }
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perms, int[] results) {
        super.onRequestPermissionsResult(code, perms, results);
        if (code != REQ_PERM) return;
        for (int i = 0; i < perms.length; i++) {
            if (perms[i].equals(Manifest.permission.CAMERA)
                    && results[i] == PackageManager.PERMISSION_GRANTED) {
                startAll();
                return;
            }
        }
        Toast.makeText(this, "相机权限被拒绝，无法预览", Toast.LENGTH_LONG).show();
    }

    private void startAll() {
        if (accel != null) sm.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST);
        if (gyro != null) sm.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST);
        if (mag != null) sm.registerListener(this, mag, SensorManager.SENSOR_DELAY_FASTEST);
        if (lm != null) {
            try {
                lm.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 0f, locListener);
                lm.registerGnssStatusCallback(gnssCb, ui);
            } catch (SecurityException e) {
                Log.e(TAG, "no location permission", e);
            }
        }
        if (camViewBack.isAvailable()) backCam.open();
        if (camViewFront.isAvailable()) frontCam.open();
        ui.post(uiTick);
    }

    private CamCtl targetCam() {
        return ctlTarget.getCheckedRadioButtonId() == R.id.targetFront ? frontCam : backCam;
    }

    private static String focusLabelFor(float dist) {
        if (dist <= 0f) return "∞";
        return String.format(Locale.US, "%.0f cm", 100f / dist);
    }

    private void syncPanel() {
        CamCtl c = targetCam();
        boolean ok = c != null && c.ready;
        zoomSeek.setEnabled(ok && c.zoomOk);
        focusCb.setEnabled(ok && c.focusOk);
        focusSeek.setEnabled(ok && c.focusOk && c.manualFocus);
        evSeek.setEnabled(ok && c.evOk);
        if (!ok) return;
        zoomSeek.setProgress(Math.round((c.zoomRatio - c.zoomMin) / (c.zoomMax - c.zoomMin) * 100f));
        zoomLabel.setText(String.format(Locale.US, "%.1fx", c.zoomRatio));
        focusCb.setChecked(c.manualFocus);
        focusSeek.setProgress(Math.round((1f - c.focusDist / c.minFocusDist) * 100f));
        focusLabel.setText(focusLabelFor(c.focusDist));
        evSeek.setProgress(c.evSteps - c.evMin);
        evLabel.setText(String.format(Locale.US, "%+d", c.evSteps));
    }

    private static Size pickPreviewSize(StreamConfigurationMap map) {
        List<Size> sizes = new ArrayList<>(Arrays.asList(
                map.getOutputSizes(SurfaceTexture.class)));
        sizes.sort(new Comparator<Size>() {
            @Override
            public int compare(Size a, Size b) {
                long areaA = (long) a.getWidth() * a.getHeight();
                long areaB = (long) b.getWidth() * b.getHeight();
                return Long.compare(areaB, areaA);
            }
        });
        for (Size s : sizes) {
            if (s.getWidth() == 1280 && s.getHeight() == 720) return s;
        }
        for (Size s : sizes) {
            if (s.getWidth() <= 1280 && s.getHeight() <= 720) return s;
        }
        return sizes.get(0);
    }

    // ---------------------------------------------------------------- sensors
    @Override
    public void onSensorChanged(SensorEvent e) {
        switch (e.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                aX = e.values[0];
                aY = e.values[1];
                aZ = e.values[2];
                aCount++;
                break;
            case Sensor.TYPE_GYROSCOPE:
                gX = e.values[0];
                gY = e.values[1];
                gZ = e.values[2];
                gCount++;
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                mX = e.values[0];
                mY = e.values[1];
                mZ = e.values[2];
                mCount++;
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor s, int acc) {
    }

    // ---------------------------------------------------------------- ui
    private static String camStatus(CamCtl c, String name) {
        if (c == null || c.camId == null) return name + "未启动";
        if (!c.ready) return name + "启动中...";
        return String.format(Locale.US, "%s %s\n%d x %d\n%.1f fps",
                name, c.camId, c.previewSize.getWidth(), c.previewSize.getHeight(),
                c.fpsFrames * 10.0);
    }

    private void updateViews() {
        camInfoBack.setText(camStatus(backCam, "后置"));
        camInfoFront.setText(camStatus(frontCam, "前置"));
        accelX.setText(String.format(Locale.US, "ax = %+.3f", aX));
        accelY.setText(String.format(Locale.US, "ay = %+.3f", aY));
        accelZ.setText(String.format(Locale.US, "az = %+.3f", aZ));
        accelHz.setText(String.format(Locale.US, "采样率 ≈ %.0f Hz  事件 %d",
                (aCount - aCountLast) * 10.0, aCount));
        gyroX.setText(String.format(Locale.US, "gx = %+.4f", gX));
        gyroY.setText(String.format(Locale.US, "gy = %+.4f", gY));
        gyroZ.setText(String.format(Locale.US, "gz = %+.4f", gZ));
        gyroHz.setText(String.format(Locale.US, "采样率 ≈ %.0f Hz  事件 %d",
                (gCount - gCountLast) * 10.0, gCount));
        imuEvents.setText(String.format(Locale.US,
                "加速度计: %s  陀螺仪: %s",
                accel != null ? "OK" : "不可用",
                gyro != null ? "OK" : "不可用"));
        aCountLast = aCount;
        gCountLast = gCount;

        double magMag = Math.sqrt(mX * mX + mY * mY + mZ * mZ);
        magInfo.setText(String.format(Locale.US,
                "磁强计: %s\n\nmx = %+.1f µT\nmy = %+.1f µT\nmz = %+.1f µT\n"
                        + "合矢量 = %.1f µT\n\n采样率 ≈ %.0f Hz  事件 %d",
                mag != null ? "OK" : "不可用", mX, mY, mZ, magMag,
                (mCount - mCountLast) * 10.0, mCount));
        mCountLast = mCount;

        String gpsText;
        if (hasFix) {
            long ageMs = System.currentTimeMillis() - fixT;
            gpsText = String.format(Locale.US,
                    "定位状态: 已定位\n\n纬度: %.7f\n经度: %.7f\n高度: %.1f m\n"
                            + "速度: %.2f m/s (%.1f km/h)\n航向: %.1f°\n"
                            + "精度: %.1f m\n卫星: %d/%d\n定位时间: %tH:%<tM:%<tS",
                    lat, lon, alt, speed, speed * 3.6, bearing, accuracy,
                    satsUsed, satsTotal, fixT);
        } else {
            gpsText = String.format(Locale.US,
                    "定位状态: 搜索中...\n卫星: %d/%d",
                    satsUsed, satsTotal);
        }
        gpsInfo.setText(gpsText);
    }

    @Override
    protected void onDestroy() {
        sm.unregisterListener(this);
        if (lm != null) {
            try {
                lm.removeUpdates(locListener);
                lm.unregisterGnssStatusCallback(gnssCb);
            } catch (SecurityException ignored) {
            }
        }
        if (backCam != null) backCam.close();
        if (frontCam != null) frontCam.close();
        ui.removeCallbacks(uiTick);
        super.onDestroy();
    }
}
