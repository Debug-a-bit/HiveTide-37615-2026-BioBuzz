package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "Dual Motor 5-Rotations Preset", group = "Testing")
public class TestMotorTargetPosition extends LinearOpMode {

    private DcMotorEx testMotor1 = null;
    private DcMotorEx testMotor2 = null;

    // The total tick count for 1 full rotation of your specific yellow jacket model
    private final double TICKS_PER_REV = 537.7;

    @Override
    public void runOpMode() {

        testMotor1 = hardwareMap.get(DcMotorEx.class, "test_motor1");
        testMotor2 = hardwareMap.get(DcMotorEx.class, "test_motor2");

        // 1. CLEAR OLD DATA: Set the active internal counters back to 0 on boot up
        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 2. CALCULATE TARGET STEPS: 5 full rotations * ticks per revolution
        int targetTickPosition = (int) (5.0 * TICKS_PER_REV); // Evaluates to roughly 2688 ticks

        // 3. INJECT THE TARGET COORD: Tell the objects where their destination is
        testMotor1.setTargetPosition(targetTickPosition);
        testMotor2.setTargetPosition(targetTickPosition);

        // 4. CHANGE THE BRAIN MODE: Lock them into position management mode
        testMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Targets Injected. Ready to execute.");
        telemetry.update();

        // The Red Light Stop Sign: Hold here until you tap PLAY
        waitForStart();

        // 5. ENGAGE THE SPEED CAP
        // This is a speed LIMIT, not a locked value. The hub will automatically use up
        // to 40% power to fly to the destination, and scale down smoothly as it approaches.
        testMotor1.setPower(0.40);
        testMotor2.setPower(0.40);

        // 6. THE SAFE MONITOR LOOP
        // keeps the thread alive while the motors are actively searching for their target ticks.
        // isBusy() returns true if the motor is still turning toward its destination.
        while (opModeIsActive() && (testMotor1.isBusy() || testMotor2.isBusy())) {

            // Package live tick tracking data to watch it on your handheld screen
            telemetry.addLine("Moving to position...");
            telemetry.addData("Target Tick Goal", targetTickPosition);
            telemetry.addData("Motor 1 Live Ticks", testMotor1.getCurrentPosition());
            telemetry.addData("Motor 2 Live Ticks", testMotor2.getCurrentPosition());
            telemetry.update();
        }

        // 7. ARREST THE VOLTAGE: Turn off the power once the target is reached successfully
        testMotor1.setPower(0.0);
        testMotor2.setPower(0.0);

        // Print final completion status to the dashboard panel
        telemetry.addLine("🎯 TARGET POSITION REACHED!");
        telemetry.addData("Motor 1 Final", testMotor1.getCurrentPosition());
        telemetry.addData("Motor 2 Final", testMotor2.getCurrentPosition());
        telemetry.update();

        // Wait a few seconds so you can read the final numbers before the file exits entirely
        sleep(3000);
    }
}
