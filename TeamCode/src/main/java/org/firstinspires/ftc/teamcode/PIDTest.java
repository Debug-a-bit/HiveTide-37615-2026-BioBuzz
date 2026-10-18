package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "PIDTest", group = "Testing")
public class PIDTest extends LinearOpMode {

    // Yellow Jacket placeholder ticks per revolution
    private static final double TICKS_PER_REV = 28;

    // MAX SHOOTER RPM: The max rpm the shooter can reach
    private static final double SHOOTER_MAX_RPM = 5800;
    // MAX TICKS PER SECOND: Max tpm the shooter can reach (based on max shooter rpm
    //private static final double MAX_TICKS_PER_S = SHOOTER_MAX_RPM / 60.0 * TICKS_PER_REV;   // ~2707
    // RPM STEP: How much the target RPM changes on dpad press
    private static final double RPM_STEP = 50;

    // Shooter PID values.
    static final double SHOOTER_P = 5.0;
    static final double SHOOTER_I = 0.0;
    static final double SHOOTER_D = 0.0;
    static final double SHOOTER_F = 0.0;   // 32767.0 / MAX_TICKS_PER_S

    // Default motor values
    private static final double INDEXER_FORWARD = 1.0;
    private static final double INDEXER_BACKWARD = -0.5;
    private static final double INDEXER_DEFAULT = -0.5;
    private static final double INTAKE_FORWARD = 1.0;
    private static final double INTAKE_BACKWARD = -1.0;
    private static final double INTAKE_DEFAULT = 0.0;

    // TARGET RPM: Threshold for shooter to be considered at target/max speed
    double targetRPM = 2300;
    // SHOOTER ON: Boolean used to toggle shooter on and off.
    boolean shooterOn = false;

    boolean autoIndex = false;
    boolean autoIntake = false;

    // LAST <BUTTON> Variable: Used to prevent button 'bouncing' (repetitive toggles) on hold.
    //      These ones are for the Shooter Toggle.
    boolean lastRT = false; boolean lastUp = false; boolean lastDown = false;

    //      Currently unused, just placeholders.
    // boolean lastRB = false; boolean lastLB = false; boolean lastA = false; boolean lastB = false;
    // boolean lastY = false; boolean lastX = false;

    // Variables for storing the applied power (used near bottom in button detection).
    double intakePower;
    double indexerPower;

    @Override
    public void runOpMode() {
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
        
        // Motors that don't require PID (and also brake).
        DcMotorEx[] nonPIDMotors = {
                frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, indexerMotor, intakeMotor
        };

        // This motor will run the PID and all calculations.
        DcMotorEx[] shooterDriveMotor = {
                rightShooterMotor
        };

        // This motor will use the motor.getPower() method on the shooterDriver to use the calculated PID values.
        DcMotorEx[] shooterFollowMotor = {
                leftShooterMotor
        };

        for (DcMotorEx motor : nonPIDMotors) {
            // When motor.setPower(0), have the motor fight against any external forces.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            // Adjust motor settings to fit the GoBuilda motor specs.
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

        // Set Directions (right side reversed for standard drivetrain)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse indexerMotor so the indexer wheel spins in the same direction as the power applied.
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "All 8 Motors & IMU Initialized. Ready.");
        telemetry.update();

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
            
            // Shooter toggle. Runs if trigger is pressed, only runs once due to !lastRT preventing debouncing.
            if (gamepad1.right_trigger_pressed && !lastRT) shooterOn = !shooterOn;

            // If button is pressed and the button is different from last time (prevents inf. loop).
            if (gamepad1.dpad_down && !lastUp) targetRPM += RPM_STEP;
            if (gamepad1.dpad_up && !lastDown) targetRPM -= RPM_STEP;

            // Set the current value as the last value pressed variable.
            lastRT = gamepad1.right_trigger_pressed;
            lastUp = gamepad1.dpad_down;
            lastDown = gamepad1.dpad_up;

            // Ensures that the targetRPM is never greater than the max and less than 0.
            targetRPM = Range.clip(targetRPM,0.0,SHOOTER_MAX_RPM);

            // Checks for shooter toggle, right shooter does PID, left shooter follows.
            if (shooterOn) {
                rightShooterMotor.setVelocity(rpmToTicksPerSec(targetRPM));
                //leftShooterMotor.setPower(rightShooterMotor.getPower());
            } else {
                rightShooterMotor.setPower(0);
                //leftShooterMotor.setPower(0);
            }

            // Actual found Shooter RPM.
            double measuredShooterRpm = ticksPerSecToRPM(rightShooterMotor.getVelocity());

            // Feeds in balls into the shooter if desired RPM is met.
            if (measuredShooterRpm >= targetRPM) {
                autoIndex = true;
                autoIntake = true;
            } else {
                autoIndex = false;
                autoIntake = false;
            }

            // --- INTAKE & INDEXER CONTROLS ---
            // Intake: 'A' button forwards (+1.0), 'Y' button reverse (-1.0)
            if (gamepad1.a) {
                intakePower = INTAKE_FORWARD;
            } else if (gamepad1.y) {
                intakePower = INTAKE_BACKWARD;
            } else if (autoIndex) {
                intakePower = INTAKE_FORWARD;
            } else {
                intakePower = INTAKE_DEFAULT;
            }
            intakeMotor.setPower(intakePower);

            // Indexer Priority:
            // 1. Manual override: RB feeds forward (+1.0), LB reverses (-1.0)
            // 2. Auto trigger: fires when shooter reaches and sustains max speed

            if (gamepad1.right_bumper) {
                indexerPower = INDEXER_FORWARD;
            } else if (gamepad1.left_bumper) {
                indexerPower = INDEXER_BACKWARD;
            } else if (autoIndex) {
                indexerPower = INDEXER_FORWARD; // Auto-feed ball into shooter
            } else {
                indexerPower = INDEXER_DEFAULT;
            }
            indexerMotor.setPower(indexerPower);

            // --- TELEMETRY DASHBOARD ---
            telemetry.addLine("=== DRIVETRAIN (FIELD-CENTRIC) ===");
            telemetry.addData("Heading (deg)", "%.1f (Press Options/Start to reset)", Math.toDegrees(botHeading));

            telemetry.addLine("=== SHOOTER & INDEXER ===");
            telemetry.addData("Shooter Target", "%.0f RPM (Target: %.0f)", measuredShooterRpm, targetRPM);
            //telemetry.addData("Spun Up & Stable", autoIndexActive ? "READY TO FIRE (FEEDING)" : (isCurrentlyAtSpeed ? "STABILIZING..." : "SPOOLING/OFF"));
            //telemetry.addData("Shooter Motors", "L: %.0f RPM | R: %.0f RPM", lShootRpm, rShootRpm);

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
