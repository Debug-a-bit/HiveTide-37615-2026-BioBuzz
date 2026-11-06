package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tracks where the robot is on the field, measured from the target goal.
 *
 * Field frame: the goal is at (0, 0). X = the IMU's "forward" direction, Y = 90° to its left,
 * centimetres. So the frame turns with the IMU heading reset (call reset() after one).
 *
 * Two sources, combined:
 *   - Wheel odometry (drive encoders + IMU heading): fast and smooth but drifts as the wheels slip.
 *   - Camera (goal AprilTags): exact position, but slow, a few frames late, and only when the
 *     goal is in view. Each sighting pulls the odometry back onto the truth. The lag is undone by
 *     working out where the robot was WHEN THE FRAME WAS TAKEN and adding the odometry since.
 *
 * Tuning (FTC Dashboard):
 *   1. CAMERA_*: where the camera sits from the robot's centre of rotation.
 *   2. Odometry: push the robot 120 cm forward by hand with the camera covered; adjust
 *      TICKS_PER_REV / WHEEL_DIAMETER_CM until odoForward reads 120. Same sideways for STRAFE_SCALE.
 *   3. VISION_GAIN: lower = smoother but slower to correct; 1 = jump straight to each camera reading.
 */
@Config
public class Localizer {

    // ---- Camera mounting, from the robot's centre of rotation (cm / degrees) ----
    public static double CAMERA_FORWARD_CM = 0;   // + = camera is in front of the centre
    public static double CAMERA_LEFT_CM    = 0;   // + = camera is left of the centre
    public static double CAMERA_YAW_DEG    = 0;   // + = camera is turned to the left

    // ---- Drive wheel odometry ----
    public static double TICKS_PER_REV     = 537.7;   // goBILDA 312 rpm
    public static double WHEEL_DIAMETER_CM = 10.4;    // goBILDA 104 mm mecanum
    public static double STRAFE_SCALE      = 1.0;     // mecanum strafes short: raise if strafing under-reads

    /** How much of each camera reading to take (0..1). */
    public static double VISION_GAIN = 0.5;
    /** Forget the pose after this long without seeing the goal (odometry drift). 0 = never. */
    public static double POSE_TIMEOUT_SEC = 10;
    /** Ignore camera readings further than this (cm): far tags are noisy. */
    public static double MAX_VISION_RANGE_CM = 380;

    private static final int HISTORY_SIZE = 64;   // ~1 s of loops

    private final ThriftyCamera camera;

    // Odometry: integrated forever, never corrected (history stays consistent)
    private double odoX = 0, odoY = 0;
    private int[] lastWheels = null;
    private double lastHeadingDeg = 0;

    // Correction added to odometry to give the field pose
    private double offsetX = 0, offsetY = 0;
    private boolean hasPose = false;
    private final ElapsedTime sinceSeen = new ElapsedTime();

    private final long[] historyNanos = new long[HISTORY_SIZE];
    private final double[] historyHeading = new double[HISTORY_SIZE];
    private final double[] historyX = new double[HISTORY_SIZE];
    private final double[] historyY = new double[HISTORY_SIZE];
    private int historyNext = 0, historyCount = 0;

    private double headingDeg = 0;

    public Localizer(ThriftyCamera camera) {
        this.camera = camera;
    }

    /**
     * Call once per loop, after camera.update() and drive.updateHeading().
     *
     * @param wheels      drive encoder positions (FL, BL, FR, BR), from drive.getWheelPositions()
     * @param headingDeg  IMU heading (+ = counter-clockwise)
     */
    public void update(int[] wheels, double headingDeg) {
        this.headingDeg = headingDeg;
        updateOdometry(wheels, headingDeg);
        recordHistory(System.nanoTime(), headingDeg);

        if (camera.isNewTarget() && camera.getTargetRange() <= MAX_VISION_RANGE_CM) {
            applyVision();
        } else if (hasPose && POSE_TIMEOUT_SEC > 0 && sinceSeen.seconds() > POSE_TIMEOUT_SEC) {
            hasPose = false;
        }
    }

    private void updateOdometry(int[] wheels, double headingDeg) {
        if (lastWheels == null) {
            lastWheels = wheels.clone();
            lastHeadingDeg = headingDeg;
            return;
        }
        double cmPerTick = Math.PI * WHEEL_DIAMETER_CM / TICKS_PER_REV;
        double dFL = (wheels[0] - lastWheels[0]) * cmPerTick;
        double dBL = (wheels[1] - lastWheels[1]) * cmPerTick;
        double dFR = (wheels[2] - lastWheels[2]) * cmPerTick;
        double dBR = (wheels[3] - lastWheels[3]) * cmPerTick;
        lastWheels = wheels.clone();

        // Mecanum: FL/BR = forward + strafe, BL/FR = forward - strafe (matches FieldCentricDrive)
        double forward = (dFL + dBL + dFR + dBR) / 4;
        double left    = -(dFL - dBL - dFR + dBR) / 4 * STRAFE_SCALE;

        // Rotate into the field using the heading halfway through the move
        double h = Math.toRadians(lastHeadingDeg + angleWrap(headingDeg - lastHeadingDeg) / 2);
        odoX += forward * Math.cos(h) - left * Math.sin(h);
        odoY += forward * Math.sin(h) + left * Math.cos(h);
        lastHeadingDeg = headingDeg;
    }

    /** Pose from the camera, moved forward to now with the odometry since the frame was taken. */
    private void applyVision() {
        int frame = historyIndexAt(camera.getTargetFrameNanos());
        double frameHeading = frame >= 0 ? historyHeading[frame] : headingDeg;
        double frameOdoX    = frame >= 0 ? historyX[frame] : odoX;
        double frameOdoY    = frame >= 0 ? historyY[frame] : odoY;

        // Goal relative to the camera (forward / left), then to the robot centre
        double camForward = camera.getTargetForward();
        double camLeft    = -camera.getTargetRight();
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        double goalForward = CAMERA_FORWARD_CM + camForward * Math.cos(yaw) - camLeft * Math.sin(yaw);
        double goalLeft    = CAMERA_LEFT_CM    + camForward * Math.sin(yaw) + camLeft * Math.cos(yaw);

        // Into the field frame; the goal is the origin, so the robot is minus that
        double h = Math.toRadians(frameHeading);
        double robotX = -(goalForward * Math.cos(h) - goalLeft * Math.sin(h));
        double robotY = -(goalForward * Math.sin(h) + goalLeft * Math.cos(h));

        double newOffsetX = robotX - frameOdoX;
        double newOffsetY = robotY - frameOdoY;
        if (hasPose) {
            offsetX += VISION_GAIN * (newOffsetX - offsetX);
            offsetY += VISION_GAIN * (newOffsetY - offsetY);
        } else {
            offsetX = newOffsetX;   // first sighting: take it as is
            offsetY = newOffsetY;
            hasPose = true;
        }
        sinceSeen.reset();
    }

    /** Forget the pose, e.g. after the IMU heading is reset (the field frame turns with it). */
    public void reset() {
        hasPose = false;
        historyCount = 0;
        lastWheels = null;
    }

    /** True once the goal has been seen (and not timed out). */
    public boolean hasPose()          { return hasPose; }
    /** Robot position from the goal (cm, field frame). Only valid while hasPose(). */
    public double getX()              { return odoX + offsetX; }
    public double getY()              { return odoY + offsetY; }
    /** Straight-line floor distance from the robot centre to the goal (cm). */
    public double getGoalDistance()   { return Math.hypot(getX(), getY()); }
    /** IMU heading the robot should have to face the goal (degrees). */
    public double getGoalHeadingDeg() { return Math.toDegrees(Math.atan2(-getY(), -getX())); }
    public double getSecondsSinceSeen() { return sinceSeen.seconds(); }
    /** Raw wheel odometry (no camera), for tuning TICKS_PER_REV / STRAFE_SCALE. */
    public double getOdoX()           { return odoX; }
    public double getOdoY()           { return odoY; }

    private void recordHistory(long nanos, double headingDeg) {
        historyNanos[historyNext] = nanos;
        historyHeading[historyNext] = headingDeg;
        historyX[historyNext] = odoX;
        historyY[historyNext] = odoY;
        historyNext = (historyNext + 1) % HISTORY_SIZE;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
    }

    /** History slot recorded closest to the given time, or -1 if there's none. */
    private int historyIndexAt(long nanos) {
        int best = -1;
        long bestDiff = Long.MAX_VALUE;
        for (int i = 0; i < historyCount; i++) {
            long diff = Math.abs(historyNanos[i] - nanos);
            if (diff < bestDiff) {
                bestDiff = diff;
                best = i;
            }
        }
        return best;
    }

    static double angleWrap(double deg) {
        while (deg > 180)   deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
