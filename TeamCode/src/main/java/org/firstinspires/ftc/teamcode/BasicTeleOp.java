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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "BasicTeleop", group = "Testing")
public class BasicTeleOp extends LinearOpMode {

    // Yellow Jacket placeholder ticks per revolution (change if needed)
    private static final double TICKS_PER_REV = 28;

    // TARGET RPM: Threshold for shooter to be considered at target/max speed
    private static final double SHOOTER_TARGET_RPM = 300;

    // Time (seconds) the shooter must continuously maintain target RPM before auto-feeding
    private static final double SHOOTER_STABILIZE_TIME_SEC = 0.35;

    @Override
    public void runOpMode() {
        // 1. Hardware Mapping using DcMotorEx for velocity/RPM reading
        DcMotorEx frontLeftMotor    = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor     = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor   = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor    = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        DcMotorEx rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        DcMotorEx leftShooterMotor  = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        DcMotorEx indexerMotor      = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        DcMotorEx intakeMotor       = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        //PID values for Shooter
        double P = 5;
        double I = 0;
        double D = 0;
        double F = 0;

        // 2. IMU Initialization for Field-Centric Drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // 3. Set Directions (right side reversed for standard drivetrain)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



// 4. Set velocity in encoder ticks per second
        // Reverse indexerMotor so the indexer wheel spins inwards/forwards
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 4. Reset and set run mode for all 8 motors
        DcMotorEx[] shooterMotors = {
                rightShooterMotor, leftShooterMotor};

        DcMotorEx[] nonShooterMotors = {
                frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, indexerMotor, intakeMotor
                };

        for (DcMotorEx motor : shooterMotors) {
            PIDFCoefficients pidfGains = new PIDFCoefficients(P, I, D, F);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfGains);
           motor.getMotorType().setTicksPerRev(TICKS_PER_REV);
        }

        // Reverse left shooter so both shooter wheels spin inwards/forwards together
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : nonShooterMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "All 8 Motors & IMU Initialized. Ready.");
        telemetry.update();

        // Timer for confirming shooter stability at target speed
        ElapsedTime shooterSpoolTimer = new ElapsedTime();
        boolean wasAtSpeed = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
            double flRpm = (frontLeftMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double blRpm = (backLeftMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double frRpm = (frontRightMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double brRpm = (backRightMotor.getVelocity() / TICKS_PER_REV) * 60.0;

            double rShootRpm = (rightShooterMotor.getVelocity(AngleUnit.RADIANS) / (2 * Math.PI)) * 60.0;
            double lShootRpm = (leftShooterMotor.getVelocity(AngleUnit.RADIANS) / (2 * Math.PI)) * 60.0;
            double avgShooterRpm = (Math.abs(rShootRpm) + Math.abs(lShootRpm)) / 2.0;

            double indexRpm  = (indexerMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double intakeRpm = (intakeMotor.getVelocity() / TICKS_PER_REV) * 60.0;

            // --- SHOOTER CONTROL & AUTO-INDEX LOGIC ---
            // Shooter: Right Trigger throttles forward (0.0 to 1.0), Left Trigger reverses (anti-jam)


            // Check if user is actively spinning up the shooter forward
            boolean isShootingRequested = gamepad1.right_trigger > 0.1;
            if (isShootingRequested) {
                rightShooterMotor.setVelocity(SHOOTER_TARGET_RPM * (2 * Math.PI) / 60,AngleUnit.RADIANS);
                leftShooterMotor.setVelocity(SHOOTER_TARGET_RPM * (2 * Math.PI) / 60,AngleUnit.RADIANS);
            } else {
                rightShooterMotor.setVelocity(0);
                leftShooterMotor.setVelocity(0);
            };
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
            //boolean autoIndexActive = isCurrentlyAtSpeed && (shooterSpoolTimer.seconds() >= SHOOTER_STABILIZE_TIME_SEC);

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
            if (gamepad1.right_bumper) {
                indexerPower = 1.0;
            } else if (gamepad1.left_bumper) {
                indexerPower = -1.0;
//            } else if (autoIndexActive) {
//                indexerPower = 1.0; // Auto-feed ball into shooter
            } else {
                indexerPower = 0.0;
            }
            indexerMotor.setPower(indexerPower);

            // --- TELEMETRY DASHBOARD ---
            telemetry.addLine("=== DRIVETRAIN (FIELD-CENTRIC) ===");
            telemetry.addData("Heading (deg)", "%.1f (Press Options/Start to reset)", Math.toDegrees(botHeading));
            telemetry.addData("FL", "Pwr: %.2f | RPM: %.0f", frontLeftPower, flRpm);
            telemetry.addData("FR", "Pwr: %.2f | RPM: %.0f", frontRightPower, frRpm);
            telemetry.addData("BL", "Pwr: %.2f | RPM: %.0f", backLeftPower, blRpm);
            telemetry.addData("BR", "Pwr: %.2f | RPM: %.0f", backRightPower, brRpm);

            telemetry.addLine("=== SHOOTER & INDEXER ===");
            telemetry.addData("Shooter Target", "%.0f RPM (Target: %.0f)", avgShooterRpm, SHOOTER_TARGET_RPM);
            //telemetry.addData("Spun Up & Stable", autoIndexActive ? "READY TO FIRE (FEEDING)" : (isCurrentlyAtSpeed ? "STABILIZING..." : "SPOOLING/OFF"));
            telemetry.addData("Shooter Motors", "L: %.0f RPM | R: %.0f RPM", lShootRpm, rShootRpm);
            telemetry.addData("Indexer (Auto/RB/LB)", "Pwr: %.2f | RPM: %.0f", indexerPower, indexRpm);
            telemetry.addData("Intake (A/Y)", "Pwr: %.2f | RPM: %.0f", intakePower, intakeRpm);

            telemetry.update();
        }
    }
}
