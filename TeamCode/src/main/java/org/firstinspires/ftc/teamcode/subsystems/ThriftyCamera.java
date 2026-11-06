package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
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
 * Each goal is a cluster of 4 tags (RED SCORING = IDs 30-33, BLUE SCORING = 42-45).
 * The SDK solves the cluster as one target, so the "target" here is the TARGET_GOAL cluster.
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
    public static double DASHBOARD_FPS  = 10;   // 0 = don't stream to Dashboard (saves Wi-Fi)

    public static final String RED_GOAL  = "RED SCORING";
    public static final String BLUE_GOAL = "BLUE SCORING";
    /** Tag cluster to target (set from the alliance picked at init). */
    public static String TARGET_GOAL = BLUE_GOAL;

    /** Higher = faster processing but shorter detection range (1 to 3 is typical). */
    public static float DECIMATION = 2;

    // ---- Manual exposure: short exposure = less motion blur while driving. 0 = auto ----
    public static int EXPOSURE_MS = 0;
    public static int GAIN = 250;

    // ---- Lens calibration (pixels, for RESOLUTION_WIDTH x RESOLUTION_HEIGHT) ----
    public static double FX = 548.3391729, FY = 548.1896237, CX = 330.2344345, CY = 271.3700637;
    // Distortion, OpenCV 8-coefficient rational model (k1, k2, p1, p2, k3, k4, k5, k6). All 0 = no undistortion.
    public static double K1 = -0.01651677248, K2 = -0.03461525769, P1 = -0.0002241824697, P2 = 0.0002172363125, K3 = 0.03839363111, K4 = 0.0008629032551, K5 = 0.001556644143, K6 = -0.001602928766;

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private boolean exposureSet = false;
    private double focalPx = 0;

    private List<AprilTagDetection> detections = new ArrayList<>();
    private AprilTagClusterDetection target = null;
    private boolean newTarget = false;

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
        portal = portalBuilder.addProcessor(aprilTag).build();

        if (DASHBOARD_FPS > 0) FtcDashboard.getInstance().startCameraStream(portal, DASHBOARD_FPS);
    }

    /** Grabs the newest detections and picks out the goal. Call once per loop. */
    public void update() {
        newTarget = false;
        if (portal == null) return;

        if (!exposureSet && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            applyExposure();
            exposureSet = true;
        }

        List<AprilTagDetection> fresh = aprilTag.getFreshDetections();
        if (fresh == null) return;   // no new frame since last loop: keep the last result
        detections = fresh;

        target = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata != null && cluster.ftcPose != null && TARGET_GOAL.equals(cluster.metadata.name)) {
                target = cluster;
            }
        }
        newTarget = target != null;
    }

    private void applyExposure() {
        if (EXPOSURE_MS <= 0) return;
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        portal.getCameraControl(GainControl.class).setGain(GAIN);
    }

    public boolean isConnected()     { return portal != null && portal.getCameraState() != VisionPortal.CameraState.ERROR; }
    public boolean hasTarget()       { return target != null; }
    public int getTagCount()         { return detections.size(); }
    /** True only on the loop a new camera frame containing the goal arrived. */
    public boolean isNewTarget()     { return newTarget; }
    /** System.nanoTime() when the frame with the goal was captured. */
    public long getTargetFrameNanos() { return target != null ? target.frameAcquisitionNanoTime : 0; }
    /** Straight-line distance from the camera lens to the goal (cm). */
    public double getTargetRange()   { return target != null ? target.ftcPose.range : 0; }
    /** How far to turn to face the goal (degrees, + = goal is to the left). */
    public double getTargetBearing() { return target != null ? target.ftcPose.bearing : 0; }
    /** How far the goal is rotated from facing the camera square-on (degrees). */
    public double getTargetYaw()     { return target != null ? target.ftcPose.yaw : 0; }
    /** Distance straight ahead of the camera to the goal (cm), ignoring sideways offset. */
    public double getTargetForward() { return target != null ? target.ftcPose.y : 0; }
    /** Distance to the right of the camera to the goal (cm). */
    public double getTargetRight()   { return target != null ? target.ftcPose.x : 0; }
    /** Horizontal focal length (pixels) the AprilTag maths is using; 0 if no camera. */
    public double getFocalPx()       { return focalPx; }
    public double getFps()           { return portal != null ? portal.getFps() : 0; }

    /** Everything seen in the latest frame (single tags and clusters). */
    public List<AprilTagDetection> getDetections() { return detections; }

    public void close() {
        if (portal == null) return;
        if (DASHBOARD_FPS > 0) FtcDashboard.getInstance().stopCameraStream();
        portal.close();
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
