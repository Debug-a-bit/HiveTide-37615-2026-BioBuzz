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
import org.firstinspires.ftc.teamcode.subsystems.SideServos;
import org.firstinspires.ftc.teamcode.util.VoltageMonitor;

/**
 * Controls (gamepad 1)
 *   Left stick      drive / strafe (field-centric)
 *   Right stick X   turn
 *   Options         reset heading ("forward" = way the robot faces now)
 *   Right bumper    hold to shoot: spin up, then auto-feed all balls once at speed
 *   Left bumper     hold to spin up only (no auto-feed)
 *   Y               reverse intake and indexer
 *   Right trigger   manual intake feed
 *   Left trigger    eject (reverse intake and clear mag full)
 *   A               side servos to angle A
 *   B               side servos to angle B
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
        SideServos sideServos   = new SideServos(hardwareMap);
        VoltageMonitor battery  = new VoltageMonitor(hardwareMap);

        telemetry.addData("Status", "Initialized. Ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        sideServos.goToA();   // servos don't move until first commanded

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
                    shooting, // Currently shooting Variable
                    shooting && shooter.isReadyToFeed(), // Is the shooter reved up check.
                    gamepad1.left_trigger_pressed, // Eject Keybind
                    gamepad1.right_trigger_pressed, // Manual Intake Keybind
                    gamepad1.y); // Run intake and index backwards.

            // ================= SIDE SERVOS =================
            if (gamepad1.a)      sideServos.goToA();
            else if (gamepad1.b) sideServos.goToB();

            // ================= TELEMETRY (graph these in Dashboard) =================
            telemetry.addData("heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("targetRPM", shooter.getTargetRpm());
            telemetry.addData("rawRPM", shooter.getRawRpm());
            telemetry.addData("filteredRPM", shooter.getRpm());
            telemetry.addData("shooterPower", shooter.getPower());
            telemetry.addData("readyToFeed", shooter.isReadyToFeed() ? 1 : 0);
            telemetry.addData("volleyActive", feeder.isVolleyActive() ? 1 : 0);
            telemetry.addData("beamBroken", feeder.isBeamBroken());
            telemetry.addData("magazineFull", feeder.isMagazineFull());
            telemetry.addData("sideServoDeg", sideServos.getAngle());
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
