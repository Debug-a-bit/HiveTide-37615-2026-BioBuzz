package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Four Motor RPM Test", group = "Testing")
public class TestMotor extends LinearOpMode {

    // 1. DECLARATION: Create the empty containers for your four extended motors
    private DcMotorEx testMotor1 = null;
    private DcMotorEx testMotor2 = null;
    private DcMotorEx testMotor3 = null;
    private DcMotorEx testMotor4 = null;

    // Change this decimal if you know your exact Yellow Jacket ratio's TPR count!
    // (537.7 is the standard placeholder for the popular 435 RPM model)
    private final double TICKS_PER_REV = 537.7;

    @Override
    public void runOpMode() {

        // 2. THE HANDSHAKE: Pair your code variables to your screen configuration names
        testMotor1 = hardwareMap.get(DcMotorEx.class, "test_motor1");
        testMotor2 = hardwareMap.get(DcMotorEx.class, "test_motor2");
        testMotor3 = hardwareMap.get(DcMotorEx.class, "test_motor3");
        testMotor4 = hardwareMap.get(DcMotorEx.class, "test_motor4");

        // Clear out old encoder tick counters and boot them into raw power mode
        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "All 4 Handshakes Passed. Ready.");
        telemetry.update();

        // The Red Light: Pause the script right here until you tap PLAY on the screen
        waitForStart();

        // 3. THE HEARTBEAT LOOP (Runs 50+ times a second once you hit PLAY)
        while (opModeIsActive()) {

            // READ INPUTS:
            // Motor 1: Left Stick Y (up is negative on Android joysticks)
            // Motor 2: Right Stick Y
            // Motor 3: Left Trigger (0.0 to 1.0)
            // Motor 4: Right Trigger (0.0 to 1.0)
            double m1Power = -gamepad1.left_stick_y;
            double m2Power = -gamepad1.right_stick_y;
            double m3Power = gamepad1.left_trigger;
            double m4Power = gamepad1.right_trigger;

            // SEND VOLTAGE: Set powers to all 4 motors
            testMotor1.setPower(m1Power);
            testMotor2.setPower(m2Power);
            testMotor3.setPower(m3Power);
            testMotor4.setPower(m4Power);

            // LIVE POWER TARGETS
            telemetry.addData("Motor 1 Power Target", m1Power);
            telemetry.addData("Motor 2 Power Target", m2Power);
            telemetry.addData("Motor 3 Power Target", m3Power);
            telemetry.addData("Motor 4 Power Target", m4Power);

            // 🧮 VELOCITY & RPM MATH (Ticks / Second -> RPM)
            double tps1 = testMotor1.getVelocity();
            double rpm1 = (tps1 / TICKS_PER_REV) * 60.0;

            double tps2 = testMotor2.getVelocity();
            double rpm2 = (tps2 / TICKS_PER_REV) * 60.0;

            double tps3 = testMotor3.getVelocity();
            double rpm3 = (tps3 / TICKS_PER_REV) * 60.0;

            double tps4 = testMotor4.getVelocity();
            double rpm4 = (tps4 / TICKS_PER_REV) * 60.0;

            // 📊 LIVE DASHBOARD ANALYTICS
            telemetry.addLine("--- MOTOR 1 SPEEDWAY ---");
            telemetry.addData("Ticks per Second (M1)", tps1);
            telemetry.addData("🎯 REAL-TIME RPM 1", rpm1);

            telemetry.addLine("--- MOTOR 2 SPEEDWAY ---");
            telemetry.addData("Ticks per Second (M2)", tps2);
            telemetry.addData("🎯 REAL-TIME RPM 2", rpm2);

            telemetry.addLine("--- MOTOR 3 SPEEDWAY ---");
            telemetry.addData("Ticks per Second (M3)", tps3);
            telemetry.addData("🎯 REAL-TIME RPM 3", rpm3);

            telemetry.addLine("--- MOTOR 4 SPEEDWAY ---");
            telemetry.addData("Ticks per Second (M4)", tps4);
            telemetry.addData("🎯 REAL-TIME RPM 4", rpm4);

            telemetry.update();
        }
    }
}
