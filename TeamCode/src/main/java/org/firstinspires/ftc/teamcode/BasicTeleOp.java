package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "BasicTeleop", group = "Testing")
public class BasicTeleOp extends LinearOpMode {

    // Yellow Jacket placeholder ticks per revolution
    private static final double TICKS_PER_REV = 28;

    // MAX SHOOTER RPM: The max rpm the shooter can reach
    private static final double SHOOTER_MAX_RPM = 5800;

    // Shooter PID values.
    static final double SHOOTER_P = 5.0;
    static final double SHOOTER_I = 0.0;
    static final double SHOOTER_D = 0.0;
    static final double SHOOTER_F = 0.0;
    public static double INDEXER_IDLE_POWER = -1.0;
    public static double SHOOTER_POWER = 0.8;

    // TARGET RPM: Threshold for shooter to be considered at target/max speed
    public static double SHOOTER_TARGET_RPM = 2300;

    // Time (seconds) the shooter must continuously maintain target RPM before auto-feeding
    private static final double SHOOTER_STABILIZE_TIME_SEC = 0.35;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Hardware Mapping using DcMotorEx for velocity/RPM reading
        DcMotorEx frontLeftMotor    = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor     = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor   = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor    = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        DcMotorEx rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        DcMotorEx leftShooterMotor  = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        DcMotorEx indexerMotor      = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        DcMotorEx intakeMotor       = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // IMU Initialization for Field-Centric Drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Set Directions (right side reversed for standard drivetrain)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse indexerMotor so the indexer wheel spins inwards/forwards
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // This motor will run the PID and all calculations.
        DcMotorEx[] shooterDriveMotor = {
                rightShooterMotor
        };

        // This motor will use the motor.getPower() method on the shooterDriver to use the calculated PID values.
        DcMotorEx[] shooterFollowMotor = {
                leftShooterMotor
        };

        // Reset and set run mode for all 8 motors
        DcMotorEx[] nonShooterMotors = {
                frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, indexerMotor, intakeMotor};

        for (DcMotorEx motor : nonShooterMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotorEx motor : shooterFollowMotor) {
            // When motor.setPower(0), have the motor go with any external forces.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setDirection(DcMotorSimple.Direction.REVERSE); // Only if shooterFollowMotor is left.
        }

        for (DcMotorEx motor : shooterDriveMotor ){
            // Create a Motor Config Type Based on the current motor settings (on hub)
            MotorConfigurationType shooter = motor.getMotorType().clone();
            // Adjust motor settings to fit the GoBilda motor specs.
            shooter.setTicksPerRev(TICKS_PER_REV);
            shooter.setGearing(1);
            shooter.setMaxRPM(SHOOTER_MAX_RPM);
            shooter.setAchieveableMaxRPMFraction(1.0);
            // Sets the shooterDriveMotor to the newly created shooter type.
            motor.setMotorType(shooter);

            // When motor.setPower(0), have the motor go with any external forces.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // PID Values for the shooter velocity.
            motor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        }

        // Reverse left shooter so both shooter wheels spin inwards/forwards together
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "All 8 Motors & IMU Initialized. Ready.");
        telemetry.update();

        // Timer for confirming shooter stability at target speed
        ElapsedTime shooterSpoolTimer = new ElapsedTime();
        boolean wasAtSpeed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- FIELD-CENTRIC DRIVE KINEMATICS ---
            double y  = -gamepad1.left_stick_y;  // Forward / backward (inverted)
            double x  = gamepad1.left_stick_x;   // Strafe left / right
            double rx = gamepad1.right_stick_x;  // Turn left / right

            // Reset field heading if options (or start) button is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate movement vector by robot heading to make it field-centric
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            // Normalize powers so no motor exceeds 1.0 while preserving ratios
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower  = (rotY - rotX + rx) / denominator;
            double backLeftPower   = (rotY + rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            // Apply power to drivetrain
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- VELOCITY & RPM CALCULATIONS ---
            double rShootRpm = (rightShooterMotor.getVelocity())/TICKS_PER_REV * 60;
            double lShootRpm = (leftShooterMotor.getVelocity())/TICKS_PER_REV * 60;
            double avgShooterRpm = (Math.abs(rShootRpm) + Math.abs(lShootRpm)) / 2.0;

            // Checks for shooter toggle, right shooter does PID, left shooter follows.
            boolean rightShooterOn = gamepad1.right_bumper;
            if (rightShooterOn) {
                rightShooterMotor.setVelocity(rpmToTicksPerSec(SHOOTER_TARGET_RPM));
                //leftShooterMotor.setPower(rightShooterMotor.getPower());
            } else {
                rightShooterMotor.setPower(0);
                //leftShooterMotor.setPower(0);
            }

            boolean leftShooterOn = gamepad1.left_bumper;
            if (leftShooterOn) {
//                rightShooterMotor.setVelocity(rpmToTicksPerSec(SHOOTER_TARGET_RPM));
                leftShooterMotor.setPower(0.8);
            } else {
//                rightShooterMotor.setPower(0);
                leftShooterMotor.setPower(0);
            }


//            boolean isCurrentlyAtSpeed = isShootingRequested && (avgShooterRpm >= SHOOTER_TARGET_RPM);
//
//            // Check if shooter has reached and stayed at speed
//            if (isCurrentlyAtSpeed) {
//                if (!wasAtSpeed) {
//                    shooterSpoolTimer.reset();
//                    wasAtSpeed = true;
//                }
//            } else {
//                wasAtSpeed = false;
//            }

            // Auto-trigger condition: held at target speed for the stabilization duration
//            boolean autoIndexActive = isCurrentlyAtSpeed && (shooterSpoolTimer.seconds() >= SHOOTER_STABILIZE_TIME_SEC);

            // --- INTAKE & INDEXER CONTROLS ---
            // Intake: 'A' button forwards (+1.0), 'Y' button reverse (-1.0)
            double intakePower = 0.0;
            if (gamepad1.a) {
                intakePower = 1.0;
            } else if (gamepad1.y) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // Indexer Priority:
            // 1. Manual override: RB feeds forward (+1.0), LB reverses (-1.0)
            // 2. Auto trigger: fires when shooter reaches and sustains max speed
            double indexerPower;
            if (gamepad1.right_trigger_pressed) {
                indexerPower = 1.0;
            } else if (gamepad1.left_trigger_pressed) {
                indexerPower = -1.0;
//            } else if (autoIndexActive) {
//                indexerPower = 1.0; // Auto-feed ball into shooter
            } else {
                indexerPower = INDEXER_IDLE_POWER;
            }
            indexerMotor.setPower(indexerPower);

            // --- TELEMETRY DASHBOARD ---
            telemetry.addLine("=== DRIVETRAIN (FIELD-CENTRIC) ===");
            telemetry.addData("Heading (deg)", "%.1f (Press Options/Start to reset)", Math.toDegrees(botHeading));

            telemetry.addLine("=== SHOOTER & INDEXER ===");
//            telemetry.addData("Shooter Target", "%.0f RPM (Target: %.0f)", avgShooterRpm, SHOOTER_TARGET_RPM);
            //telemetry.addData("Spun Up & Stable", autoIndexActive ? "READY TO FIRE (FEEDING)" : (isCurrentlyAtSpeed ? "STABILIZING..." : "SPOOLING/OFF"));
            telemetry.addData("rightRPM", rShootRpm);
            telemetry.addData("leftRPM", lShootRpm);
            telemetry.addData("avgRPM", avgShooterRpm);
            telemetry.update();
        }
    }
    static double rpmToTicksPerSec  (double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }
    static double ticksPerSecToRPM (double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
}
