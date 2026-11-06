package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SideServos;
import org.firstinspires.ftc.teamcode.subsystems.ThriftyCamera;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.VoltageMonitor;

/**
 * During init: X = blue alliance, B = red alliance (picks which goal to aim at).
 *
 * Controls (gamepad 1)
 *   Left stick      drive / strafe (field-centric)
 *   Right stick X   turn (ignored while auto-aim is turning toward the goal, or D-pad up is held)
 *   D-pad up        hold to turn to HIVE_TILT_HEADING_DEG (sticks still drive / strafe)
 *   Options         reset heading ("forward" = way the robot faces now)
 *   Right bumper    hold to shoot: auto-aim at the goal, spin up, then auto-feed all balls once at speed and aimed.
 *                   Aims from the robot's tracked field position, so the goal only has to be seen
 *                   once (within Localizer.POSE_TIMEOUT_SEC); it doesn't have to stay in view.
 *   Left bumper     hold to spin up only (no auto-feed)
 *   Y               reverse intake and indexer (without clearing magfull)
 *   Right trigger   manual intake feed
 *   Left trigger    eject (reverse intake and clear mag full)
 *   A               side servos to angle A
 *   B               side servos to angle B
 *
 * The controller rumbles when the magazine becomes full.
 *
 * Every loop is also written to /sdcard/FIRST/logs/teleop_<date>_<time>.csv
 * (pull with: adb pull /sdcard/FIRST/logs).
 */
@Config
@TeleOp(name = "BasicTeleop", group = "Testing")
public class BasicTeleOp extends LinearOpMode {

    /** How often (ms) the Dashboard graph gets a new point. Lower = smoother graph, more Wi-Fi traffic. */
    public static int DASHBOARD_INTERVAL_MS = 20;
    /** How long (ms) the controller rumbles when the magazine fills. 0 = off. */
    public static int FULL_RUMBLE_MS = 400;
    /** Save every loop to a CSV on the Control Hub. */
    public static boolean LOG_TO_FILE = false;
    /** Heading D-pad up turns to (degrees, + = counter-clockwise; 0 = the way the robot faced at heading reset). */
    public static double HIVE_TILT_HEADING_DEG = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(DASHBOARD_INTERVAL_MS);   // default is 100 ms
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Read all motor/encoder values from each hub in one go per loop (much faster loops)
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
        Shooter shooter         = new Shooter(hardwareMap);
        Feeder feeder           = new Feeder(hardwareMap);
        SideServos sideServos   = new SideServos(hardwareMap);
        VoltageMonitor battery  = new VoltageMonitor(hardwareMap);
        ThriftyCamera camera    = new ThriftyCamera(hardwareMap);
        Localizer localizer     = new Localizer(camera);
        AutoAim aim             = new AutoAim(localizer);

        DataLogger log = LOG_TO_FILE ? new DataLogger("teleop",
                "targetRPM", "rawRPM", "filteredRPM", "shooterPower", "loadComp", "shotsFired",
                "readyToFeed", "volleyActive", "beamBroken", "magazineFull", "jamEjecting", "intakeVelocity",
                "sideServoDeg", "servoPowered", "batteryVoltage",
                "tagCount", "goalSeen", "goalRangeCm", "goalBearingDeg", "goalYawDeg", "aiming", "aimErrorDeg", "loopMs") : null;

        // Pick alliance (which goal to aim at) until START is pressed
        while (opModeInInit()) {
            if (gamepad1.x)      ThriftyCamera.TARGET_GOAL = ThriftyCamera.BLUE_GOAL;
            else if (gamepad1.b) ThriftyCamera.TARGET_GOAL = ThriftyCamera.RED_GOAL;

            telemetry.addData("Status", "Initialized. Ready.");
            telemetry.addData("Camera", camera.isConnected() ? "connected" : "NOT FOUND (check config name)");
            telemetry.addData("Aiming at", ThriftyCamera.TARGET_GOAL + "   (X = blue, B = red)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        sideServos.goToB();   // servos don't move until first commanded

        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastA = false, lastB = false;
        boolean lastMagazineFull = false;
        boolean lastAligning = false;

        while (opModeIsActive()) {
            double batteryVoltage = battery.update();

            boolean shooting = gamepad1.right_bumper;
            boolean align = gamepad1.x;

            // ================= CAMERA, POSITION & AIM =================
            camera.update();
            double heading = drive.updateHeading();
            if (gamepad1.options) {
                drive.resetHeading();
                localizer.reset();   // the field frame turns with the heading, so the old pose is wrong
            }
            localizer.update(drive.getWheelPositions(), heading);
            aim.update(shooting, heading);

            // ================= DRIVE =================
            // Auto-aim at the goal wins; otherwise D-pad up turns to HIVE_TILT_HEADING_DEG; otherwise the stick
            boolean aligning = gamepad1.dpad_up && !aim.isAiming();
            if (aligning && !lastAligning) drive.resetAlign();   // fresh controller memory on each press
            lastAligning = aligning;

            if (align) {
                drive.alignWithHiveTilt(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.toRadians(drive.getHeadingDegrees() + camera.getTargetBearing()));
            }

            if (aim.isAiming()) {
                drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, aim.getTurn());
            } else if (aligning) {
                drive.alignWithHiveTilt(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        Math.toRadians(HIVE_TILT_HEADING_DEG));
            } else {
                drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // ================= SHOOTER =================
            boolean spinOnly = gamepad1.left_bumper;

            if (shooting || spinOnly)       shooter.spinUp();
            else if (feeder.isBackingOff()) shooter.setOpenLoopPower(Feeder.SHOOTER_BACKOFF_POWER);  // help push a ball back off the beam
            else                            shooter.stop();
            // Push harder while balls are going through (feeder state is from last loop, ~1 loop behind)
            shooter.setFeeding(feeder.isVolleyActive() || (shooting && gamepad1.right_trigger_pressed));
            shooter.update(batteryVoltage);

            // ================= INTAKE & INDEXER =================
            feeder.update(
                    shooting, // Currently shooting Variable
                    shooting && shooter.isReadyToFeed() && aim.isReadyToShoot(), // Shooter revved up and aim settled on the goal.
                    gamepad1.left_trigger_pressed, // Eject Keybind
                    gamepad1.right_trigger_pressed, // Manual Intake Keybind
                    gamepad1.y); // Run intake and index backwards.

            // Rumble once when the magazine becomes full
            if (feeder.isMagazineFull() && !lastMagazineFull && FULL_RUMBLE_MS > 0) gamepad1.rumble(FULL_RUMBLE_MS);
            lastMagazineFull = feeder.isMagazineFull();

            // ================= SIDE SERVOS =================
            // Only on the press, so holding the button doesn't keep re-powering the servos
            if (gamepad1.a && !lastA)      sideServos.goToA();
            else if (gamepad1.b && !lastB) sideServos.goToB();
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            sideServos.update();

            // ================= TELEMETRY (graph these in Dashboard) =================
            telemetry.addData("heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("alignError (deg)", aligning ? drive.getAlignErrorDegrees() : 0);
            telemetry.addData("targetRPM", shooter.getTargetRpm());
            telemetry.addData("rawRPM", shooter.getRawRpm());
            telemetry.addData("filteredRPM", shooter.getRpm());
            telemetry.addData("shooterPower", shooter.getPower());
            telemetry.addData("readyToFeed", shooter.isReadyToFeed() ? 1 : 0);
            telemetry.addData("loadComp", shooter.isLoadCompensating() ? 1 : 0);
            telemetry.addData("shotsFired", shooter.getShotsFired());
            telemetry.addData("volleyActive", feeder.isVolleyActive() ? 1 : 0);
            telemetry.addData("beamBroken", feeder.isBeamBroken());
            telemetry.addData("magazineFull", feeder.isMagazineFull());
            telemetry.addData("jamEjecting", feeder.isJamEjecting() ? 1 : 0);
            telemetry.addData("intakeVelocity", feeder.getIntakeVelocity());
            telemetry.addData("sideServoDeg", sideServos.getAngle());
            telemetry.addData("servoPowered", sideServos.isPowered() ? 1 : 0);
            telemetry.addData("batteryVoltage", batteryVoltage);
            telemetry.addData("cameraFps", camera.getFps());
            telemetry.addData("tagCount", camera.getTagCount());
            telemetry.addData("goalSeen", camera.hasTarget() ? 1 : 0);
            telemetry.addData("goalRange (cm)", camera.getTargetRange());
            telemetry.addData("goalBearing (deg)", camera.getTargetBearing());
            telemetry.addData("goalYaw (deg)", camera.getTargetYaw());
            telemetry.addData("poseValid", localizer.hasPose() ? 1 : 0);
            telemetry.addData("poseX (cm)", localizer.getX());
            telemetry.addData("poseY (cm)", localizer.getY());
            telemetry.addData("goalDistance (cm)", localizer.getGoalDistance());
            telemetry.addData("odoX (cm)", localizer.getOdoX());
            telemetry.addData("odoY (cm)", localizer.getOdoY());
            telemetry.addData("aiming", aim.isAiming() ? 1 : 0);
            telemetry.addData("aimed", aim.isAimed() ? 1 : 0);
            telemetry.addData("aimReady", aim.isReadyToShoot() ? 1 : 0);
            telemetry.addData("aimError (deg)", aim.getErrorDeg());
            double loopMs = loopTimer.milliseconds();
            telemetry.addData("loopMs", loopMs);
            loopTimer.reset();

            if (log != null) {
                log.log(shooter.getTargetRpm(), shooter.getRawRpm(), shooter.getRpm(), shooter.getPower(),
                        shooter.isLoadCompensating() ? 1 : 0, shooter.getShotsFired(),
                        shooter.isReadyToFeed() ? 1 : 0, feeder.isVolleyActive() ? 1 : 0,
                        feeder.isBeamBroken() ? 1 : 0, feeder.isMagazineFull() ? 1 : 0,
                        feeder.isJamEjecting() ? 1 : 0, feeder.getIntakeVelocity(),
                        sideServos.getAngle(), sideServos.isPowered() ? 1 : 0,
                        batteryVoltage, camera.getTagCount(), camera.hasTarget() ? 1 : 0,
                        camera.getTargetRange(), camera.getTargetBearing(), camera.getTargetYaw(),
                        aim.isAiming() ? 1 : 0, aim.getErrorDeg(), loopMs);
            }
            telemetry.update();
        }

        drive.stop();
        shooter.stop();
        shooter.update(battery.get());
        feeder.stop();
        camera.close();
        if (log != null) log.close();
    }
}
