package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorEx.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorEx.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorEx.Direction.FORWARD);
        c.backRightDirection.set(DcMotorEx.Direction.REVERSE);
    });

    public static MecanumImuOdometry.Config localizerConfig = new MecanumImuOdometry.Config();
    static {
        localizerConfig.logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;      // match your hub
        localizerConfig.usb  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;  // match your hub
        localizerConfig.forwardInchesPerTick = 1.0;  // replace after calibration (48 / x reading)
        localizerConfig.strafeInchesPerTick  = 1.0;  // replace after calibration (48 / y reading)
    }

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Paste the Java output from the AutoTune Foresight Tuner here.
        // Pedro will refuse to create a Follower until these values are set.
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new DriveEncoderIMULocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
