package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
// Ctrl+click "Localizer" in PinpointLocalizer to find its exact package

public class DriveEncoderIMULocalizer implements Localizer {
    private final MecanumImuOdometry odo;

    public DriveEncoderIMULocalizer(HardwareMap h, MecanumImuOdometry.Config config) {
        odo = new MecanumImuOdometry(h, config);
    }

    @Override
    public void setPose(Pose pose) {

    }

    @Override
    public MotionState state() {
        return null;
    }

    @Override
    public void update() {

    }

    @Override
    public void reset() {

    }

    // ...generated methods go here
}