package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * ThriftyBot ThriftyCam (USB webcam) running the SDK AprilTag processor.
 *
 * Add the camera in the Robot Configuration as a "Webcam" named CAMERA_NAME.
 * If it isn't configured or plugged in, the rest of the robot still runs:
 * isConnected() is false and every getter returns "no tag".
 *
 * BIOBUZZ goal: each alliance's HIVE has two CELLS; the upward-facing one is the one to score in.
 * Each CELL has a cluster of 4 tags on its BOTTOM, lying flat and facing the floor (red: 30-33 and
 * 34-37, blue: 38-41 and 42-45). Only the upward-facing CELL's tags can be seen from the floor, so
 * the target is whichever of our alliance's clusters is in view. The SDK solves each cluster as one
 * target.
 *
 * Because the tags lie flat, their tilt in the image gives the true vertical. That is used to work out
 * where the goal is across the floor, whatever angle the camera is pitched or rolled at.
 *
 * The live camera image is sent to FTC Dashboard (Camera view) with tag outlines drawn on it.
 *
 * Lens: uses the calibrated intrinsics in FX/FY/CX/CY and distortion K1..K6/P1/P2 (OpenCV 8-coefficient
 * model). Redo the calibration if the lens or resolution changes.
 */
@Config
public class ThriftyCamera {

    public static String CAMERA_NAME = "Webcam 1";
    public static int RESOLUTION_WIDTH  = 640;
    public static int RESOLUTION_HEIGHT = 480;
    public static double DASHBOARD_FPS  = 1;   // 0 = don't stream to Dashboard (saves Wi-Fi)
    /**
     * True = camera is mounted upside down (image rotated 180°): flips the sign of bearing, yaw and the
     * fallback left/right. The floor position (used for aiming) works out which way is up from the tags.
     */
    public static boolean UPSIDE_DOWN = true;

    public static final String RED  = "RED";
    public static final String BLUE = "BLUE";
    /** Alliance whose CELLS to target (set at init). Matches any cluster whose name starts with it. */
    public static String ALLIANCE = RED;

    /** Higher = faster processing but shorter detection range (1 to 3 is typical). */
    public static float DECIMATION = 2;

    // ---- Manual exposure: short exposure = less motion blur while driving. 0 = auto ----
    // With AUTO_BRIGHTNESS on, these are only the starting point.
    public static int EXPOSURE_MS = 1;
    public static int GAIN = 1;   // this camera accepts 0 to 4; each step is big (4 washes the image out)

    // ---- Auto brightness: tunes exposure (then gain) toward TARGET_BRIGHTNESS ----
    /** False = always use the fixed EXPOSURE_MS and GAIN above. */
    public static boolean AUTO_BRIGHTNESS = true;
    /** False = tune only until lockBrightness() (called at START), then hold. True = keep adjusting all match. */
    public static boolean AUTO_BRIGHTNESS_IN_MATCH = false;
    /** Average image brightness to aim for (0 = black, 255 = white). */
    public static double TARGET_BRIGHTNESS = 110;
    /** Close enough to the target to leave the settings alone. */
    public static double BRIGHTNESS_TOLERANCE = 10;
    /** Longest exposure auto brightness may use (longer = more motion blur); gain goes up after this. */
    public static double MAX_AUTO_EXPOSURE_MS = 5;
    /** Frames to wait after a change before judging it (the camera takes a few frames to apply one). */
    public static int SETTLE_FRAMES = 3;

    // ---- Lens calibration (pixels, for RESOLUTION_WIDTH x RESOLUTION_HEIGHT) ----
    public static double FX = 548.3391729, FY = 548.1896237, CX = 330.2344345, CY = 271.3700637;
    // Distortion, OpenCV 8-coefficient rational model (k1, k2, p1, p2, k3, k4, k5, k6). All 0 = no undistortion.
    public static double K1 = -0.01651677248, K2 = -0.03461525769, P1 = -0.0002241824697, P2 = 0.0002172363125, K3 = 0.03839363111, K4 = 0.0008629032551, K5 = 0.001556644143, K6 = -0.001602928766;

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private boolean exposureSet = false;
    private double focalPx = 0;

    // ---- Auto brightness state ----
    private final BrightnessMeter meter = new BrightnessMeter();
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private boolean autoBrightnessActive = false;
    private boolean brightnessLocked = false;
    private double exposureMs = 0;
    private int gain = 0;
    private long lastChangeFrame = 0;

    private List<AprilTagDetection> detections = new ArrayList<>();
    private AprilTagClusterDetection target = null;
    private boolean newTarget = false;

    // Goal position across the floor (cm), from computeFloorPosition()
    private double floorForward = 0, floorRight = 0, tagHeight = 0, cameraTiltDeg = 0;

    public ThriftyCamera(HardwareMap hardwareMap) {
        WebcamName webcam;
        try {
            webcam = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        } catch (IllegalArgumentException e) {
            RobotLog.ww("ThriftyCamera", "No webcam named \"%s\" in the config; vision disabled", CAMERA_NAME);
            return;
        }
        if (!webcam.isAttached()) {   // in the config but not plugged in
            RobotLog.ww("ThriftyCamera", "Webcam \"%s\" is not plugged in; vision disabled", CAMERA_NAME);
            return;
        }

        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setLensIntrinsics(FX, FY, CX, CY);
        focalPx = FX;
        aprilTag = tagBuilder.build();
        aprilTag.setDecimation(DECIMATION);

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);   // less USB bandwidth than YUY2
        // The SDK's AprilTag pose ignores lens distortion, so straighten the frame before it sees it
        if (K1 != 0 || K2 != 0 || P1 != 0 || P2 != 0 || K3 != 0 || K4 != 0 || K5 != 0 || K6 != 0) {
            portalBuilder.addProcessor(new Undistorter());
        }
        portal = portalBuilder.addProcessor(meter).addProcessor(aprilTag).build();

        if (DASHBOARD_FPS > 0) FtcDashboard.getInstance().startCameraStream(portal, DASHBOARD_FPS);
    }

    /** Grabs the newest detections and picks out the goal. Call once per loop. */
    public void update() {
        newTarget = false;
        if (portal == null) return;

        if (!exposureSet && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            exposureControl = portal.getCameraControl(ExposureControl.class);
            gainControl = portal.getCameraControl(GainControl.class);
            applyExposure();
            exposureSet = true;
        }
        if (exposureSet) updateBrightness();

        List<AprilTagDetection> fresh = aprilTag.getFreshDetections();
        if (fresh == null) return;   // no new frame since last loop: keep the last result
        detections = fresh;

        // Either of our CELLS (only the upward-facing one's tags are visible); the nearest if both are
        target = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null || cluster.ftcPose == null || !cluster.metadata.name.startsWith(ALLIANCE)) continue;
            if (target == null || cluster.ftcPose.range < target.ftcPose.range) target = cluster;
        }
        newTarget = target != null;
        if (target != null) computeFloorPosition();
    }

    /**
     * The tags lie flat facing the floor, so the line straight out of them is the true vertical.
     * Taking that out of the camera's reading leaves where the goal is across the floor, so camera
     * pitch (it looks up at the tags) and roll (including being mounted upside down) don't skew the
     * distance or the aim.
     */
    private void computeFloorPosition() {
        // Camera axes (as ftcPose): x = image right, y = straight out of the lens, z = image up
        double tx = target.ftcPose.x, ty = target.ftcPose.y, tz = target.ftcPose.z;
        floorForward  = ty;         // fallbacks: the plain camera reading
        floorRight    = flip(tx);
        tagHeight     = 0;
        cameraTiltDeg = 0;
        if (target.rawPose == null || target.rawPose.R == null) return;

        // Tag normal (the R matrix's 3rd column, raw axes x right / y down / z out of the lens) = vertical
        MatrixF r = target.rawPose.R;
        double ux = r.get(0, 2), uy = r.get(2, 2), uz = -r.get(1, 2);
        double length = Math.sqrt(ux * ux + uy * uy + uz * uz);
        if (length < 1e-6) return;
        ux /= length; uy /= length; uz /= length;
        double height = tx * ux + ty * uy + tz * uz;   // camera-to-tag-plane distance = height of the tags
        if (height < 0) { ux = -ux; uy = -uy; uz = -uz; height = -height; }   // point up, toward the tags

        // Lens direction levelled onto the floor (camera pointing near straight up: can't level it)
        double fx = -uy * ux, fy = 1 - uy * uy, fz = -uy * uz;
        double fLength = Math.sqrt(fx * fx + fy * fy + fz * fz);
        if (fLength < 0.3) return;
        fx /= fLength; fy /= fLength; fz /= fLength;
        // The camera's real right across the floor: levelled lens direction x up. Taken from the true
        // vertical, so it's right whichever way up the camera is mounted (no UPSIDE_DOWN needed)
        double rx = fy * uz - fz * uy, ry = fz * ux - fx * uz, rz = fx * uy - fy * ux;

        double hx = tx - height * ux, hy = ty - height * uy, hz = tz - height * uz;   // goal, flattened to the floor
        floorForward  = hx * fx + hy * fy + hz * fz;
        floorRight    = hx * rx + hy * ry + hz * rz;
        tagHeight     = height;
        cameraTiltDeg = Math.toDegrees(Math.asin(Range.clip(uy, -1, 1)));
    }

    /**
     * Fixed EXPOSURE_MS and GAIN. Always sets both: the camera keeps its last exposure and gain between
     * runs, even across OpModes.
     */
    private void applyExposure() {
        if (EXPOSURE_MS <= 0) {
            // This camera has no plain "Auto"; AperturePriority is its auto-exposure
            exposureControl.setMode(ExposureControl.Mode.AperturePriority);
        } else {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        gainControl.setGain(GAIN);
        exposureMs = EXPOSURE_MS;
        gain = GAIN;
    }

    /**
     * Auto brightness, one step per call: too dark = longer exposure up to MAX_AUTO_EXPOSURE_MS, then more
     * gain; too bright = shorter exposure, then less gain once exposure is at its minimum. Changing one
     * thing at a time, in that order, means it settles instead of bouncing between gain steps.
     */
    private void updateBrightness() {
        if (!AUTO_BRIGHTNESS) {
            if (autoBrightnessActive) applyExposure();   // switched off: back to the fixed settings
            autoBrightnessActive = false;
            return;
        }
        double minExposureMs = exposureControl.getMinExposure(TimeUnit.MICROSECONDS) / 1000.0;
        double maxExposureMs = Math.max(MAX_AUTO_EXPOSURE_MS, minExposureMs);
        if (!autoBrightnessActive) {   // start from the fixed settings
            exposureMs = Range.clip(EXPOSURE_MS, minExposureMs, maxExposureMs);
            gain = GAIN;
            setExposureAndGain();
            lastChangeFrame = meter.getFrameCount();
            autoBrightnessActive = true;
        }
        if (brightnessLocked && !AUTO_BRIGHTNESS_IN_MATCH) return;

        long frame = meter.getFrameCount();
        if (frame < lastChangeFrame + SETTLE_FRAMES) return;   // last change hasn't shown up yet
        double measured = Math.max(meter.getBrightness(), 1);
        if (Math.abs(TARGET_BRIGHTNESS - measured) <= BRIGHTNESS_TOLERANCE) return;

        // Brightness is roughly proportional to exposure, so scale by the ratio (limited per step)
        double ratio = TARGET_BRIGHTNESS / measured;
        if (measured < TARGET_BRIGHTNESS) {
            if (exposureMs < maxExposureMs)       exposureMs = Math.min(maxExposureMs, exposureMs * Range.clip(ratio, 1.05, 1.5));
            else if (gain < gainControl.getMaxGain()) gain++;
            else return;   // as bright as it's allowed to go
        } else {
            if (exposureMs > minExposureMs)       exposureMs = Math.max(minExposureMs, exposureMs * Range.clip(ratio, 0.67, 0.95));
            else if (gain > gainControl.getMinGain()) gain--;
            else return;   // as dark as it can go
        }
        setExposureAndGain();
        lastChangeFrame = frame;
    }

    private void setExposureAndGain() {
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(Math.round(exposureMs * 1000), TimeUnit.MICROSECONDS);
        gainControl.setGain(gain);
    }

    /** Stops auto brightness adjusting (unless AUTO_BRIGHTNESS_IN_MATCH). Call at START. */
    public void lockBrightness() { brightnessLocked = true; }

    /** Average brightness of the latest frame (0 = black, 255 = white). */
    public double getBrightness()   { return meter.getBrightness(); }
    /** Exposure the camera is set to (ms); 0 = the camera's own auto exposure. */
    public double getExposureMs()   { return exposureMs; }
    public int getGain()            { return gain; }

    /** One line for telemetry: brightness against target, and the settings in use. */
    public String getBrightnessStatus() {
        if (portal == null) return "no camera";
        if (!exposureSet) return "waiting for camera";
        String mode = !AUTO_BRIGHTNESS ? "fixed"
                : brightnessLocked && !AUTO_BRIGHTNESS_IN_MATCH ? "auto, locked" : "auto, tuning";
        return String.format("%.0f (target %.0f)  exposure %.2f ms  gain %d  [%s]",
                getBrightness(), TARGET_BRIGHTNESS, exposureMs, gain, mode);
    }

    public boolean isConnected()     { return portal != null && portal.getCameraState() != VisionPortal.CameraState.ERROR; }
    public boolean hasTarget()       { return target != null; }
    public int getTagCount()         { return detections.size(); }
    /**
     * Everything in the latest frame: clusters by name with how much of the cluster was found
     * (e.g. "BLUE SCORING 75%"), single tags by ID, or "none".
     */
    public String getTagIds() {
        StringBuilder ids = new StringBuilder();
        for (AprilTagDetection detection : detections) {
            if (ids.length() > 0) ids.append(", ");
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                ids.append(cluster.metadata != null ? cluster.metadata.name : "cluster")
                   .append(' ').append(cluster.percentClusterFound).append('%');
                if (cluster.ftcPose == null) ids.append(" (no pose)");
            } else if (detection instanceof AprilTagSingleDetection) {
                ids.append(((AprilTagSingleDetection) detection).id);
            }
        }
        return ids.length() > 0 ? ids.toString() : "none";
    }
    /** True only on the loop a new camera frame containing the goal arrived. */
    public boolean isNewTarget()     { return newTarget; }
    /** System.nanoTime() when the frame with the goal was captured. */
    public long getTargetFrameNanos() { return target != null ? target.frameAcquisitionNanoTime : 0; }
    /** Straight-line distance from the camera lens to the goal (cm). */
    public double getTargetRange()   { return target != null ? target.ftcPose.range : 0; }
    /** How far to turn to face the goal (degrees, + = goal is to the left). */
    public double getTargetBearing() { return target != null ? flip(target.ftcPose.bearing) : 0; }
    /** How far the goal is rotated from facing the camera square-on (degrees). */
    public double getTargetYaw()     { return target != null ? flip(target.ftcPose.yaw) : 0; }
    /** Distance straight ahead of the camera to the goal (cm), ignoring sideways offset. */
    public double getTargetForward() { return target != null ? target.ftcPose.y : 0; }
    /** Goal ahead of the camera measured across the floor (cm): pitch and roll taken out. */
    public double getGoalFloorForward() { return target != null ? floorForward : 0; }
    /** Goal to the right of the camera measured across the floor (cm). */
    public double getGoalFloorRight()   { return target != null ? floorRight : 0; }
    /** How far the tags are above the camera lens (cm); check against a tape measure. */
    public double getTagHeight()        { return target != null ? tagHeight : 0; }
    /** How far the lens is tipped up toward the tags (degrees), worked out from the tags. */
    public double getCameraTiltDeg()    { return target != null ? cameraTiltDeg : 0; }
    /** Horizontal focal length (pixels) the AprilTag maths is using; 0 if no camera. */
    public double getFocalPx()       { return focalPx; }
    public double getFps()           { return portal != null ? portal.getFps() : 0; }

    /** A 180° camera mounting mirrors left/right, so sideways values and turn angles change sign. */
    private static double flip(double value) { return UPSIDE_DOWN ? -value : value; }

    /** Everything seen in the latest frame (single tags and clusters). */
    public List<AprilTagDetection> getDetections() { return detections; }

    public void close() {
        if (portal == null) return;
        if (DASHBOARD_FPS > 0) FtcDashboard.getInstance().stopCameraStream();
        portal.close();
    }

    /** Measures each frame's average brightness, on a small copy so it costs almost nothing. */
    private static class BrightnessMeter implements VisionProcessor {
        private final Mat small = new Mat(), gray = new Mat();
        private volatile double brightness = 0;
        private volatile long frameCount = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.resize(frame, small, new org.opencv.core.Size(80, 60), 0, 0, Imgproc.INTER_AREA);
            Imgproc.cvtColor(small, gray, small.channels() == 4 ? Imgproc.COLOR_RGBA2GRAY : Imgproc.COLOR_RGB2GRAY);
            brightness = Core.mean(gray).val[0];
            frameCount++;   // only this thread writes it
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }

        double getBrightness() { return brightness; }
        long getFrameCount()   { return frameCount; }
    }

    /**
     * Removes lens distortion from each frame in place, so the processors added after it (AprilTag)
     * see a straight image. Keeps the same FX/FY/CX/CY, so the AprilTag intrinsics still match.
     */
    private static class Undistorter implements VisionProcessor {
        private final Mat mapX = new Mat(), mapY = new Mat(), undistorted = new Mat();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            cameraMatrix.put(0, 0, FX, 0, CX, 0, FY, CY, 0, 0, 1);
            MatOfDouble distortion = new MatOfDouble(K1, K2, P1, P2, K3, K4, K5, K6);
            // Lookup maps are built once; remapping each frame is then cheap
            Calib3d.initUndistortRectifyMap(cameraMatrix, distortion, new Mat(), cameraMatrix,
                    new org.opencv.core.Size(width, height), CvType.CV_16SC2, mapX, mapY);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.remap(frame, undistorted, mapX, mapY, Imgproc.INTER_LINEAR);
            undistorted.copyTo(frame);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }
}
