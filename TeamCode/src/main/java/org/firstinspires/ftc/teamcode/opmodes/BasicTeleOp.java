package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.VoltageMonitor;

/**
 * Controls (gamepad 1)
 *   Left stick      drive / strafe (field-centric)
 *   Right stick X   turn
 *   Options         reset heading ("forward" = way the robot faces now)
 *   Right bumper    hold to shoot: spin up, auto-feed once at speed
 *   Left bumper     hold to spin up only (no auto-feed)
 *   Y               reject: reverse intake, clear "magazine full"
 *   Right trigger   manual indexer feed
 *   Left trigger    manual indexer reverse
 */
@TeleOp(name = "BasicTeleop", group = "Testing")
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Read all motor/encoder values from each hub in one go per loop (much faster loops)
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
        Shooter shooter         = new Shooter(hardwareMap);
        Feeder feeder           = new Feeder(hardwareMap);
        VoltageMonitor battery  = new VoltageMonitor(hardwareMap);

        telemetry.addData("Status", "Initialized. Ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double batteryVoltage = battery.update();

            // ================= DRIVE =================
            if (gamepad1.options) drive.resetHeading();
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // ================= SHOOTER =================
            boolean shooting = gamepad1.right_bumper;
            boolean spinOnly = gamepad1.left_bumper;

            if (shooting || spinOnly)       shooter.spinUp();
            else if (feeder.isBackingOff()) shooter.setOpenLoopPower(Feeder.SHOOTER_BACKOFF_POWER);  // help push a ball back off the beam
            else                            shooter.stop();
            shooter.update(batteryVoltage);

            // ================= INTAKE & INDEXER =================
            feeder.update(
                    shooting,
                    shooting && shooter.isReadyToFeed(),
                    gamepad1.y,
                    gamepad1.right_trigger_pressed,
                    gamepad1.left_trigger_pressed);

            // ================= TELEMETRY (graph these in Dashboard) =================
            telemetry.addData("heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("targetRPM", shooter.getTargetRpm());
            telemetry.addData("rawRPM", shooter.getRawRpm());
            telemetry.addData("filteredRPM", shooter.getRpm());
            telemetry.addData("shooterPower", shooter.getPower());
            telemetry.addData("readyToFeed", shooter.isReadyToFeed() ? 1 : 0);
            telemetry.addData("beamBroken", feeder.isBeamBroken());
            telemetry.addData("magazineFull", feeder.isMagazineFull());
            telemetry.addData("batteryVoltage", batteryVoltage);
            telemetry.addData("loopMs", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }

        drive.stop();
        shooter.stop();
        shooter.update(battery.get());
        feeder.stop();
    }
}
