package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Two 180-degree servos mounted facing outwards on opposite sides of the robot,
 * both driving the same arm.
 *
 * Because they're mirrored, the right servo is reversed in software so the
 * same angle moves both sides the same way physically.
 * Both servos are always commanded together. All angles are in degrees.
 */
@Config
public class SideServos {

    public static final double SERVO_RANGE_DEG = 180;

    public static double ANGLE_A_DEG = 15;     // e.g. stowed
    public static double ANGLE_B_DEG = 165;   // e.g. deployed

    /**
     * Added to the right servo only, to line it up with the left.
     * Both servos drive the same arm, so any mismatch makes them fight each other.
     */
    public static double RIGHT_TRIM_DEG = 0;

    private final Servo left, right;

    private double angle = ANGLE_A_DEG;

    public SideServos(HardwareMap hardwareMap) {
        left  = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");

        right.setDirection(Servo.Direction.REVERSE);   // mirrored, so flip one side
    }

    /** Move both servos to an angle (0 to 180 degrees). */
    public void setAngle(double degrees) {
        angle = clampDeg(degrees);
        left.setPosition(toPosition(angle));
        right.setPosition(toPosition(clampDeg(angle + RIGHT_TRIM_DEG)));
    }

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
