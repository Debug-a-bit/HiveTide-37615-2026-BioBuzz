package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Full Robot Test", group = "Testing")
public class FullRobotTest extends LinearOpMode {

    // Yellow Jacket placeholder ticks per revolution (change if needed)
    private static final double TICKS_PER_REV = 537.7;

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

        // 2. Set Directions (right side reversed for standard drivetrain)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse left shooter so both shooter wheels spin inwards/forwards together
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse indexerMotor so the indexer wheel spins inwards/forwards
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Reset and set run mode for all 8 motors
        DcMotorEx[] allMotors = {
            frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
            rightShooterMotor, leftShooterMotor, indexerMotor, intakeMotor
        };

        for (DcMotorEx motor : allMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "All 8 Motors Initialized & Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- MECANUM DRIVE KINEMATICS (Gamepad 1 Sticks) ---
            double y  = -gamepad1.left_stick_y;       // Forward / backward (inverted)
            double x  = gamepad1.left_stick_x * 1.1;  // Strafe left / right (1.1 counteracts imperfect strafing)
            double rx = gamepad1.right_stick_x;       // Turn left / right

            // Normalize powers so no motor exceeds 1.0 while preserving ratios
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Apply power to drivetrain
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- MECHANISM CONTROLS ---
            // Shooter: Right Trigger spins both shooter flywheels (0.0 to 1.0)
            // Left Trigger runs in reverse if needed for clearing jams
            double shooterPower = gamepad1.right_trigger > 0 ? gamepad1.right_trigger : -gamepad1.left_trigger;
            rightShooterMotor.setPower(shooterPower);
            leftShooterMotor.setPower(shooterPower);

            // Intake: 'A' button forwards (+1.0), 'Y' button reverse (-1.0)
            double intakePower = 0.0;
            if (gamepad1.a) {
                intakePower = 1.0;
            } else if (gamepad1.y) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // Indexer: Right Bumper feeds (+1.0), Left Bumper reverses (-1.0)
            double indexerPower = 0.0;
            if (gamepad1.right_bumper) {
                indexerPower = 1.0;
            } else if (gamepad1.left_bumper) {
                indexerPower = -1.0;
            }
            indexerMotor.setPower(indexerPower);

            // --- VELOCITY & RPM CALCULATIONS ---
            double flRpm = (frontLeftMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double blRpm = (backLeftMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double frRpm = (frontRightMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double brRpm = (backRightMotor.getVelocity() / TICKS_PER_REV) * 60.0;

            double rShootRpm = (rightShooterMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double lShootRpm = (leftShooterMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double indexRpm  = (indexerMotor.getVelocity() / TICKS_PER_REV) * 60.0;
            double intakeRpm = (intakeMotor.getVelocity() / TICKS_PER_REV) * 60.0;

            // --- TELEMETRY DASHBOARD ---
            telemetry.addLine("=== DRIVETRAIN ===");
            telemetry.addData("FL", "Pwr: %.2f | RPM: %.0f", frontLeftPower, flRpm);
            telemetry.addData("FR", "Pwr: %.2f | RPM: %.0f", frontRightPower, frRpm);
            telemetry.addData("BL", "Pwr: %.2f | RPM: %.0f", backLeftPower, blRpm);
            telemetry.addData("BR", "Pwr: %.2f | RPM: %.0f", backRightPower, brRpm);

            telemetry.addLine("=== MECHANISMS ===");
            telemetry.addData("Shooter (RT/LT)", "Pwr: %.2f | L: %.0f RPM | R: %.0f RPM", shooterPower, lShootRpm, rShootRpm);
            telemetry.addData("Intake (A/Y)",    "Pwr: %.2f | RPM: %.0f", intakePower, intakeRpm);
            telemetry.addData("Indexer (RB/LB)", "Pwr: %.2f | RPM: %.0f", indexerPower, indexRpm);

            telemetry.update();
        }
    }
}
