package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Two 180-degree servos mounted facing outwards on opposite sides of the robot,
 * both driving the same arm.
 *
 * Because they're mirrored, the right servo is reversed in software so the
 * same angle moves both sides the same way physically.
 * Both servos are always commanded together. All angles are in degrees.
 *
 * Power cut (angle A only): a normal servo can't report where it is, so after a move to A we wait
 * for the time it should take to get there, then turn its signal off. With no
 * signal the servo goes limp: no holding torque, no buzzing, and the two sides
 * stop fighting each other. Any new setAngle() powers them back on.
 * Note: a limp servo won't hold the arm up against gravity or bumps.
 */
@Config
public class SideServos {

    public static final double SERVO_RANGE_DEG = 180;

    public static double ANGLE_A_DEG = 22;     // e.g. stowed
    public static double ANGLE_B_DEG = 170;   // e.g. deployed

    /**
     * Added to the right servo only, to line it up with the left.
     * Both servos drive the same arm, so any mismatch makes them fight each other.
     */
    public static double RIGHT_TRIM_DEG = 12;

    /** Turn the servos off once they should have reached angle A. They stay powered at every other angle. */
    public static boolean CUT_POWER_AT_TARGET = true;
    public static double SERVO_DEG_PER_SEC    = 200;   // how fast the arm actually moves (measure it)
    public static double EXTRA_HOLD_SEC       = 0.2;   // keep powered a bit longer to be sure it arrived

    private final Servo left, right;
    private final ElapsedTime moveTimer = new ElapsedTime();

    private double angle = ANGLE_A_DEG;
    private double moveTimeSec = 0;
    private boolean powered = false;
    private boolean firstMove = true;

    public SideServos(HardwareMap hardwareMap) {
        left  = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");

        right.setDirection(Servo.Direction.REVERSE);  // mirrored, so flip one side
    }

    /** Move both servos to an angle (0 to 180 degrees). */
    public void setAngle(double degrees) {
        double target = clampDeg(degrees);
        // Position is unknown before the first move, so allow a full-range sweep
        double travelDeg = firstMove ? SERVO_RANGE_DEG : Math.abs(target - angle);
        firstMove = false;

        angle = target;
        left.setPosition(toPosition(angle));
        right.setPosition(toPosition(clampDeg(angle + RIGHT_TRIM_DEG)));
        setPowered(true);

        moveTimeSec = travelDeg / SERVO_DEG_PER_SEC + EXTRA_HOLD_SEC;
        moveTimer.reset();
    }

    /** Call once per loop. Cuts power once the move should be finished. */
    public void update() {
        if (powered && CUT_POWER_AT_TARGET && angle == clampDeg(ANGLE_A_DEG)
                && moveTimer.seconds() >= moveTimeSec) {
            setPowered(false);
        }
    }

    private void setPowered(boolean on) {
        for (Servo servo : new Servo[]{ left, right }) {
            if (servo instanceof PwmControl) {
                if (on) ((PwmControl) servo).setPwmEnable();
                else    ((PwmControl) servo).setPwmDisable();
            }
        }
        powered = on;
    }

    public boolean isPowered() { return powered; }

    public void goToA() { setAngle(ANGLE_A_DEG); }
    public void goToB() { setAngle(ANGLE_B_DEG); }

    public double getAngle() { return angle; }

    private static double clampDeg(double degrees) {
        return Math.max(0, Math.min(SERVO_RANGE_DEG, degrees));
    }

    private static double toPosition(double degrees) {
        return degrees / SERVO_RANGE_DEG;
    }
}
