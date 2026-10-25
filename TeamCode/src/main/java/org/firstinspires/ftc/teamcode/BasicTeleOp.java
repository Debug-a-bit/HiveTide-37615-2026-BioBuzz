package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.MovingAverage;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp(name = "BasicTeleop", group = "Testing")
public class BasicTeleOp extends LinearOpMode {

    // Yellow Jacket placeholder ticks per revolution
    private static final double TICKS_PER_REV = 28;

    // MAX SHOOTER RPM: The max rpm the shooter can reach
    private static final double SHOOTER_MAX_RPM = 5800;
    private static final double TESTED_SHOOTER_MAX_RPM = 5200;

    private static final double TESTED_SHOOTER_POWER = 1.0;
    private static final double TESTED_SHOOTER_BATTERY_VOLTAGE  = 12.131;
    private static final double MIN_BATTERY_VOLTAGE = 11.3;

    private static final double TESTED_SHOOTER_POWER_PER_RPM  = TESTED_SHOOTER_POWER/TESTED_SHOOTER_MAX_RPM;


    private static final double MAX_TICKS_PER_S = SHOOTER_MAX_RPM / 60.0 * TICKS_PER_REV;   // ~2707

    // Shooter PID values.
    public static double SHOOTER_P = 0.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 11.8;
    public static double SHOOTER_S = 0.38;
    public static double SHOOTER_V = (MIN_BATTERY_VOLTAGE - SHOOTER_S)/TESTED_SHOOTER_MAX_RPM;

    public static double INDEXER_IDLE_POWER = -1.0;

    public static double INDEXER_RUNNING_POWER = 1.0;

    public static double INDEXER_REVERSE_POWER = -1.0;
    public static double INTAKE_IDLE_POWER = 0;
    public static double INTAKE_RUNNING_POWER = 1.0;

    public static double INTAKE_REVERSE_POWER = -1.0;
    public static double SHOOTER_POWER = 0.8;

    // TARGET RPM: Threshold for shooter to be considered at target/max speed
    public static double SHOOTER_TARGET_RPM = 3600;

    // Time (seconds) the shooter must continuously maintain target RPM before auto-feeding
    public static double SHOOTER_STABILIZE_TIME_SEC = 0.35;

    public static double SHOOTER_DROP_TOLERANCE_RPM = 100;

    @Override
    public void runOpMode() {

        // Send data to FTC Dashboard as well
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Mapping for motors and beam breaker
        DcMotorEx frontLeftMotor    = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor     = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor   = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor    = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        DcMotorEx rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        DcMotorEx leftShooterMotor  = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        DcMotorEx indexerMotor      = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        DcMotorEx intakeMotor       = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        DigitalChannel beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        // IMU Initialization for Field-Centric Drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // Set Directions (right side reversed for standard drivetrain)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse indexerMotor so the indexer wheel spins inwards/forwards
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        DcMotorEx[] nonShooterMotors = {
                frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, indexerMotor, intakeMotor};

        for (DcMotorEx motor : nonShooterMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // This motor will run the PID and all calculations.
        DcMotorEx[] shooterDriveMotor = {
                rightShooterMotor
        };

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
        }

        // This motor will use the motor.getPower() method on the shooterDriver to use the calculated PID values.
        DcMotorEx[] shooterFollowMotor = {
                leftShooterMotor
        };
        for (DcMotorEx motor : shooterFollowMotor) {
            // When motor.setPower(0), have the motor go with any external forces.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        boolean wasAtSpeed = false;
        ElapsedTime readyTimer = new ElapsedTime();
        boolean magFull = false;
        double intakePower = 0.0;

        MovingAverage rightRpmAvg = new MovingAverage(10);
        MovingAverage leftRpmAvg  = new MovingAverage(10);


        telemetry.addData("Status", "All 8 Motors & IMU Initialized. Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- VELOCITY & RPM CALCULATIONS ---
            double rShootRpm = rightRpmAvg.update(ticksPerSecToRPM(rightShooterMotor.getVelocity()));
            double lShootRpm = leftRpmAvg.update(ticksPerSecToRPM(leftShooterMotor.getVelocity()));
            double avgShooterRpm = (Math.abs(rShootRpm) + Math.abs(lShootRpm)) / 2.0;

            // PID
            double PID_error = SHOOTER_TARGET_RPM - rShootRpm;

            double PID_V = SHOOTER_V * SHOOTER_TARGET_RPM;
            double PID_S = Math.signum(SHOOTER_TARGET_RPM)*SHOOTER_S;
            double PID_P = SHOOTER_P * PID_error;

            double PID_calculated = PID_S + PID_V + PID_P;
            double PID_clamped = Range.clip(PID_calculated, -MIN_BATTERY_VOLTAGE, MIN_BATTERY_VOLTAGE);

// Inside your OpMode class or init() method:
            VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            double currentVoltage = voltageSensor.getVoltage();

            double PIDPower = PID_clamped/currentVoltage;




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
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            // Apply power to drivetrain
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);



            // Apply PID Values for the shooter velocity.
            rightShooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

            // Get beam breaker state
            boolean beamBreakBroken = !beamBreak.getState();
            boolean shooterOn = gamepad1.right_bumper;

            if (beamBreakBroken && !shooterOn) {
                intakeMotor.setPower(INTAKE_REVERSE_POWER);
                indexerMotor.setPower(INDEXER_REVERSE_POWER);
                rightShooterMotor.setPower(-1);
                leftShooterMotor.setPower(-1);
                magFull = true;
            } else if (gamepad1.y) {
                intakePower = INTAKE_REVERSE_POWER;
                magFull = false;
            } else if (!magFull) {
                intakePower = INTAKE_RUNNING_POWER;
            } else if (!shooterOn) {
                intakePower = INTAKE_IDLE_POWER;
                rightShooterMotor.setPower(0);
                leftShooterMotor.setPower(0);
            }
            intakeMotor.setPower(intakePower);

            // Checks for shooter toggle, right shooter does PID, left shooter follows.

            if (shooterOn) {
                rightShooterMotor.setPower(PIDPower);
                leftShooterMotor.setPower(rightShooterMotor.getPower());
            } else if (gamepad1.left_bumper){
                rightShooterMotor.setPower(PIDPower);
                leftShooterMotor.setPower(PIDPower);
            } else {
                rightShooterMotor.setPower(0);
                leftShooterMotor.setPower(0);
            }



            double minRpm = ticksPerSecToRPM(rightShooterMotor.getVelocity());

            if (!shooterOn) {
                wasAtSpeed = false;                                   // reset when shooter is off
            } else if (!wasAtSpeed) {
                if (minRpm >= SHOOTER_TARGET_RPM) {
                    wasAtSpeed = true;                                // reached target: start timing
                    readyTimer.reset();
                }
            } else if (minRpm < SHOOTER_TARGET_RPM - SHOOTER_DROP_TOLERANCE_RPM) {
                wasAtSpeed = false;                                   // real drop: lose ready state
            }

            boolean readyToFeed = wasAtSpeed && readyTimer.seconds() >= SHOOTER_STABILIZE_TIME_SEC;

            double indexerPower;
            if (gamepad1.right_trigger_pressed) {
                indexerPower = INDEXER_RUNNING_POWER;                  // manual override (note: see below)
            } else if (gamepad1.left_trigger_pressed) {
                indexerPower = INDEXER_REVERSE_POWER;
            } else if (readyToFeed) {
                indexerPower = INDEXER_RUNNING_POWER;// auto-feed only when up to speed
                magFull = false;
            } else if (magFull) {
                indexerPower = 0;
            } else {
                indexerPower = INDEXER_IDLE_POWER;
            }
            indexerMotor.setPower(indexerPower);

            // --- TELEMETRY DASHBOARD ---
            telemetry.addLine("=== DRIVETRAIN (FIELD-CENTRIC) ===");
            telemetry.addData("Heading (deg)", "%.1f (Press Options/Start to reset)", Math.toDegrees(botHeading));

            telemetry.addLine("=== SHOOTER & INDEXER ===");
            telemetry.addData("rightRPM", rShootRpm);
            telemetry.addData("leftRPM", lShootRpm);
            telemetry.addData("rightShooterPower", rightShooterMotor.getPower());
            telemetry.addData("avgRPM", avgShooterRpm);
            telemetry.addData("targetRPM", SHOOTER_TARGET_RPM);
            telemetry.addData("minRpm", minRpm);
            telemetry.addData("beamBreak", beamBreakBroken);
            telemetry.addData("readyToFeed", readyToFeed ? 1 : 0);

            telemetry.addData("PIDPower", PIDPower);

            telemetry.addData("batteryVoltage", currentVoltage);

            telemetry.update();
        }
    }

    // Unit Conversion Functions
    static double rpmToTicksPerSec  (double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }
    static double ticksPerSecToRPM (double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
}
