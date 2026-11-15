package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turns the robot to face the goal (rotation only; the driver keeps full control of translation).
 *
 * Aims only while the camera can see the goal: each sighting gives the IMU heading that faces the
 * goal (Localizer), and the IMU (fast, no lag) steers to it. Goal out of view = no steering.
 *
 * Tuning (FTC Dashboard): raise kP until it snaps on quickly without wobbling, then raise
 * MIN_TURN until it can creep the last couple of degrees without stalling short.
 */
@Config
public class AutoAim {

    public static boolean ENABLED = true;

    public static double kP            = 0.06   ;   // turn power per degree of error
    public static double MIN_TURN      = 0.06;   // smallest turn power that actually moves the robot
    public static double MAX_TURN      = 0.6;
    public static double TOLERANCE_DEG = 1.5;    // "aimed" when within this; stops turning

    /** True = the shooter fires out of the robot's back (same side as the camera), so the back faces the goal. */
    public static boolean SHOOTS_BACKWARDS = true;
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
    private boolean holding = false;   // fired this press: no more turning until shoot is released

    public AutoAim(Localizer localizer) {
        this.localizer = localizer;
    }

    /**
     * Call once per loop, after localizer.update().
     *
     * @param active      true while the driver wants to aim (e.g. holding shoot)
     * @param headingDeg  current IMU heading (+ = counter-clockwise)
     * @param firing      true while balls are being fed: the aim holds still (no turning) until
     *                    shoot is released, so the robot doesn't twitch between shots
     */
    public void update(boolean active, double headingDeg, boolean firing) {
        // Goal not in view (or no camera): never steer, driver keeps the stick
        aiming = ENABLED && active && (localizer.hasGoal() || (holding && firing));
        if (!aiming) {
            turn = 0;
            holding = false;
            onTarget.reset();
            return;
        }

        double shooterOffset = SHOOTS_BACKWARDS ? 180 : 0;
        error = Localizer.angleWrap(localizer.getGoalHeadingDeg() + shooterOffset + AIM_OFFSET_DEG - headingDeg);

        // Firing: turn motors off and stay off (stick turn stays ignored too) until shoot is released
        holding = holding || firing;
        if (holding) {
            turn = 0;
            return;
        }

        if (Math.abs(error) > TOLERANCE_DEG) onTarget.reset();

        double output = Math.abs(error) <= TOLERANCE_DEG ? 0 : kP * error + MIN_TURN * Math.signum(error);
        // error + = goal is counter-clockwise, but drive's turn + = clockwise
        turn = -Range.clip(output, -MAX_TURN, MAX_TURN);
    }

    /** Turn power to use instead of the driver's stick: only valid while isAiming(). */
    public double getTurn()        { return turn; }
    /** True while the aim has control of turning (steering, or holding still while firing). */
    public boolean isAiming()      { return aiming; }
    /** True while firing: the aim is holding still with the turn motors off. */
    public boolean isHolding()     { return holding; }
    /** True while steering and within TOLERANCE_DEG of the goal. */
    public boolean isAimed()       { return aiming && Math.abs(error) <= TOLERANCE_DEG; }
    public double getErrorDeg()    { return aiming ? error : 0; }

    /**
     * True when it's OK to start feeding: aimed and held for AIM_SETTLE_SEC. If the aim isn't
     * steering (disabled, or goal not in view) the driver is aiming, so this is true.
     */
    public boolean isReadyToShoot() {
        if (!WAIT_FOR_AIM || !aiming || holding) return true;
        return isAimed() && onTarget.seconds() >= AIM_SETTLE_SEC;
    }
}
