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
 */
@Config
public class FieldCentricDrive {

    /** Boosts the sideways part of the move, because mecanum strafes slower than it drives. */
    public static double STRAFE_MULTIPLIER = 1.1;

    private final DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private final IMU imu;
    private double headingRad = 0;

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

    /**
     * @param x  strafe   (+ = right)
     * @param y  forward  (+ = forward; pass -gamepad.left_stick_y)
     * @param rx turn     (+ = clockwise)
     */
    public void drive(double x, double y, double rx) {
        headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
