package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Where the goal is from the robot, measured directly by the camera. No field position is tracked
 * or assumed: everything here is only valid while the goal is in view.
 *
 * The camera reading is a few frames late. To undo that, the IMU heading is remembered for each
 * loop, and the goal direction is worked out from the heading the robot had WHEN THE FRAME WAS TAKEN.
 *
 * Tuning (FTC Dashboard): CAMERA_*: where the camera sits from the robot's centre of rotation.
 */
@Config
public class Localizer {

    // ---- Camera mounting, from the robot's centre of rotation (cm / degrees) ----
    public static double CAMERA_FORWARD_CM = -7;  // + = camera is in front of the centre
    public static double CAMERA_LEFT_CM    = -8;  // + = camera is left of the centre
    public static double CAMERA_YAW_DEG    = 180; // + = camera is turned to the left (180 = faces backwards)

    /** Ignore camera readings further than this (cm): far tags are noisy. */
    public static double MAX_VISION_RANGE_CM = 380;

    private static final int HISTORY_SIZE = 64;   // ~1 s of loops

    private final ThriftyCamera camera;

    private boolean hasGoal = false;
    private double goalHeadingDeg = 0;

    private final long[] historyNanos = new long[HISTORY_SIZE];
    private final double[] historyHeading = new double[HISTORY_SIZE];
    private int historyNext = 0, historyCount = 0;

    public Localizer(ThriftyCamera camera) {
        this.camera = camera;
    }

    /**
     * Call once per loop, after camera.update() and drive.updateHeading().
     *
     * @param headingDeg  IMU heading (+ = counter-clockwise)
     */
    public void update(double headingDeg) {
        recordHistory(System.nanoTime(), headingDeg);

        hasGoal = camera.hasTarget() && camera.getTargetRange() <= MAX_VISION_RANGE_CM;
        if (!hasGoal) return;
        if (camera.isNewTarget()) {
            int frame = historyIndexAt(camera.getTargetFrameNanos());
            double frameHeading = frame >= 0 ? historyHeading[frame] : headingDeg;
            goalHeadingDeg = angleWrap(frameHeading
                    + Math.toDegrees(Math.atan2(goalLeftFromCamera(), goalForwardFromCamera())));
        }
    }

    // Goal (the CELL, straight above its tags) relative to the robot centre across the floor
    // (forward / left, cm), from the camera's latest sighting
    private double goalForwardFromCamera() {
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        return CAMERA_FORWARD_CM + camera.getGoalFloorForward() * Math.cos(yaw) + camera.getGoalFloorRight() * Math.sin(yaw);
    }

    private double goalLeftFromCamera() {
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        return CAMERA_LEFT_CM + camera.getGoalFloorForward() * Math.sin(yaw) - camera.getGoalFloorRight() * Math.cos(yaw);
    }

    /** Forget the goal heading, e.g. after the IMU heading is reset (it is measured in the IMU's frame). */
    public void reset() {
        hasGoal = false;
        historyCount = 0;
    }

    /** True while the camera can see the goal (within MAX_VISION_RANGE_CM). */
    public boolean hasGoal()          { return hasGoal; }
    /** IMU heading the robot should have to face the goal (degrees). Only valid while hasGoal(). */
    public double getGoalHeadingDeg() { return goalHeadingDeg; }
    /**
     * Distance from the robot centre to the goal centre (cm) from the camera alone. Only updates
     * while the goal is in view; 0 when it isn't.
     */
    public double getVisionGoalDistance() {
        return camera.hasTarget() ? Math.hypot(goalForwardFromCamera(), goalLeftFromCamera()) : 0;
    }

    private void recordHistory(long nanos, double headingDeg) {
        historyNanos[historyNext] = nanos;
        historyHeading[historyNext] = headingDeg;
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
