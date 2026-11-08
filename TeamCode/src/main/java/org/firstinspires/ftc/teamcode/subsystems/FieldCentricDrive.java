package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric mecanum drive.
 * "Forward" on the stick always means away from the driver, whichever way the robot faces.
 * Also has a heading-align mode: the sticks still control x/y, but rotation is automatic.
 */
@Config
public class FieldCentricDrive {

    /** Boosts the sideways part of the move, because mecanum strafes slower than it drives. */
    public static double STRAFE_MULTIPLIER = 1.1;

    // ---- Heading-align tuning (live-editable from FTC Dashboard). All are starting guesses. ----
    /** How hard to turn per radian of error. Higher = faster, more aggressive. */
    public static double ALIGN_KP = 1.0;
    /** The "brake": reduces the turn when the error is shrinking quickly. */
    public static double ALIGN_KD = 0.05;
    /** Speed limit on auto-turning (0 to 1). */
    public static double ALIGN_MAX_TURN = 0.6;
    /** Within this many degrees of the target counts as "close enough" and we stop turning. */
    public static double ALIGN_TOLERANCE_DEG = 2.0;

    private final DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private final IMU imu;
    private double headingRad = 0;

    // ---- Memory the align controller keeps between loops ----
    private double alignLastError = 0;
    private long alignLastTimeNs = 0;
    private boolean alignFirstLoop = true;
    private double alignErrorRad = 0;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Motors face outward on each side, so one side is reversed
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{ frontLeft, backLeft, frontRight, backRight }) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    }

    /** Reads the IMU. Call once per loop, before drive(). Returns heading in degrees (+ = counter-clockwise). */
    public double updateHeading() {
        headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return Math.toDegrees(headingRad);
    }

    /**
     * Uses the heading from the last updateHeading().
     *
     * @param x  strafe   (+ = right)
     * @param y  forward  (+ = forward; pass -gamepad.left_stick_y)
     * @param rx turn     (+ = clockwise)
     */
    public void drive(double x, double y, double rx) {
        applyDrive(x, y, rx);
    }

    /**
     * Call once on the frame the align button is first pressed,
     * so old controller memory doesn't carry over from the last alignment.
     */
    public void resetAlign() {
        alignFirstLoop = true;
        alignLastError = 0;
    }

    /**
     * Sticks control x/y as normal; rotation turns the robot to targetHeadingRad.
     * Call every loop while the align button is held. Uses the heading from the last updateHeading().
     *
     * @param x                strafe  (+ = right)
     * @param y                forward (+ = forward)
     * @param targetHeadingRad the direction to face, in radians, in the same frame as the IMU yaw
     */
    public void alignWithHiveTilt(double x, double y, double targetHeadingRad) {
        // How far off are we? Wrapped to -PI..PI so we always take the shorter way around.
        alignErrorRad = AngleUnit.normalizeRadians(targetHeadingRad - headingRad);

        // Time since the last loop, in seconds (0 on the first loop, since there's no history)
        long now = System.nanoTime();
        double dt = alignFirstLoop ? 0 : (now - alignLastTimeNs) / 1e9;
        alignLastTimeNs = now;

        // How fast is the error changing? (This is what the "brake" uses.)
        double derivative = (!alignFirstLoop && dt > 0)
                ? (alignErrorRad - alignLastError) / dt
                : 0;
        alignLastError = alignErrorRad;
        alignFirstLoop = false;

        double rx;
        if (Math.abs(alignErrorRad) < Math.toRadians(ALIGN_TOLERANCE_DEG)) {
            rx = 0; // close enough, stop turning
        } else {
            // Negated because this class treats +rx as clockwise but IMU yaw is +counterclockwise.
            // If the robot turns AWAY from the target when you test, remove the minus sign.
            rx = -(ALIGN_KP * alignErrorRad + ALIGN_KD * derivative);
            rx = Math.max(-ALIGN_MAX_TURN, Math.min(ALIGN_MAX_TURN, rx));
        }

        applyDrive(x, y, rx);
    }

    /** Wheel math shared by drive() and alignWithHiveTilt(); uses the headingRad already read. */
    private void applyDrive(double x, double y, double rx) {
        // Rotate the stick direction by -heading so it's relative to the field
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
        rotX *= STRAFE_MULTIPLIER;

        // Scale down so no wheel goes past full power, keeping the ratios
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        // Diagonal pairs: front-left & back-right get +rotX, back-left & front-right get -rotX
        frontLeft.setPower((rotY + rotX + rx) / denominator);
        backLeft.setPower((rotY - rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);
    }

    /** Makes the robot's current facing direction the new "forward". */
    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(headingRad);
    }

    /** Current align error in degrees; handy for telemetry while tuning. */
    public double getAlignErrorDegrees() {
        return Math.toDegrees(alignErrorRad);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
