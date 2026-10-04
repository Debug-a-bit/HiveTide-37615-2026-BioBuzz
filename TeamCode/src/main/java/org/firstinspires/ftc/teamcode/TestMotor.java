package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Test Motor", group = "Testing")
public class TestMotor extends LinearOpMode {
    private DcMotor testMotor1 = null;
    private DcMotor testMotor2 = null;
@Override
public void runOpMode () {
    testMotor1 = hardwareMap.get(DcMotor.class,"test_motor1");
    testMotor2 = hardwareMap.get(DcMotor.class, "test_motor2");


waitForStart();

    while (opModeIsActive()) {
        testMotor1.setPower(0.50);
        testMotor2.setPower(0.50);

    }
}
}


