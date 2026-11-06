package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turns the robot to face the goal (rotation only; the driver keeps full control of translation).
 *
 * Aims from the robot's position on the field (Localizer), not straight off the camera. So it keeps
 * pointing at the goal while driving and when the tags are out of view or blurred, and it allows
 * for the robot moving sideways. The IMU (fast, no lag) steers to the heading that faces the goal.
 *
 * Tuning (FTC Dashboard): raise kP until it snaps on quickly without wobbling, then raise
 * MIN_TURN until it can creep the last couple of degrees without stalling short.
 */
@Config
public class AutoAim {

    public static boolean ENABLED = false;

    public static double kP            = 0.02;   // turn power per degree of error
    public static double MIN_TURN      = 0.06;   // smallest turn power that actually moves the robot
    public static double MAX_TURN      = 0.6;
    public static double TOLERANCE_DEG = 1.5;    // "aimed" when within this; stops turning

    /** Added to the goal heading (+ = aim further left). Corrects a shot that pulls to one side. */
    public static double AIM_OFFSET_DEG = 0;

    /** True = auto-feed waits until the aim has settled on the goal. */
    public static boolean WAIT_FOR_AIM = true;
    /** Must stay within TOLERANCE_DEG this long before it counts as settled (stops firing while swinging through). */
    public static double AIM_SETTLE_SEC = 0.15;

    private final Localizer localizer;

    private final ElapsedTime onTarget = new ElapsedTime();
    private double error = 0;
    private double turn = 0;
    private boolean aiming = false;

    public AutoAim(Localizer localizer) {
        this.localizer = localizer;
    }

    /**
     * Call once per loop, after localizer.update().
     *
     * @param active      true while the driver wants to aim (e.g. holding shoot)
     * @param headingDeg  current IMU heading (+ = counter-clockwise)
     */
    public void update(boolean active, double headingDeg) {
        // No pose (goal not seen yet, timed out, or no camera): never steer, driver keeps the stick
        aiming = ENABLED && active && localizer.hasPose();
        if (!aiming) {
            turn = 0;
            onTarget.reset();
            return;
        }

        error = Localizer.angleWrap(localizer.getGoalHeadingDeg() + AIM_OFFSET_DEG - headingDeg);
        if (Math.abs(error) > TOLERANCE_DEG) onTarget.reset();

        double output = Math.abs(error) <= TOLERANCE_DEG ? 0 : kP * error + MIN_TURN * Math.signum(error);
        // error + = goal is counter-clockwise, but drive's turn + = clockwise
        turn = -Range.clip(output, -MAX_TURN, MAX_TURN);
    }

    /** Turn power to use instead of the driver's stick: only valid while isAiming(). */
    public double getTurn()        { return turn; }
    /** True while the aim is steering the robot. */
    public boolean isAiming()      { return aiming; }
    /** True while steering and within TOLERANCE_DEG of the goal. */
    public boolean isAimed()       { return aiming && Math.abs(error) <= TOLERANCE_DEG; }
    public double getErrorDeg()    { return aiming ? error : 0; }

    /**
     * True when it's OK to start feeding: aimed and held for AIM_SETTLE_SEC. If the aim isn't
     * steering (disabled, or position unknown) the driver is aiming, so this is true.
     */
    public boolean isReadyToShoot() {
        if (!WAIT_FOR_AIM || !aiming) return true;
        return isAimed() && onTarget.seconds() >= AIM_SETTLE_SEC;
    }
}
