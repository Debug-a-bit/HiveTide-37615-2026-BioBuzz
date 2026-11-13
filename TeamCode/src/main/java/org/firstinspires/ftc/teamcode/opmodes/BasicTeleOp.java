package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SideServos;
import org.firstinspires.ftc.teamcode.subsystems.ThriftyCamera;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.VoltageMonitor;

import java.util.List;

/**
 * During init: X = blue alliance, B = red alliance (picks which goal to aim at).
 *
 * Controls (gamepad 1)
 *   Left stick      drive / strafe (field-centric)
 *   Right stick X   turn (ignored while auto-aim is turning toward the goal, or D-pad up is held)
 *   D-pad up        hold to turn to HIVE_TILT_HEADING_DEG (sticks still drive / strafe)
 *   Options         reset heading ("forward" = way the robot faces now)
 *   Right bumper    hold to shoot: auto-aim at the goal, spin up, then auto-feed all balls once at speed and aimed.
 *                   Only aims while the camera can see the goal.
 *   Left bumper     hold to shoot without auto-aim: spin up, then auto-feed all balls once at speed
 *                   (the driver aims with the right stick).
 *   Y               reverse intake and indexer (without clearing magfull)
 *   Right trigger   manual intake feed
 *   Left trigger    eject (reverse intake and clear mag full)
 *   A               side servos to angle A
 *   B               side servos to angle B
 *
 * The controller rumbles when the magazine becomes full, and the shooter spins up then so a shot
 * doesn't have to wait for it (it coasts again once the volley empties the magazine).
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
    public static boolean LOG_TO_FILE = true;
    /** Heading D-pad up turns to (degrees, + = counter-clockwise; 0 = the way the robot faced at heading reset). */
    public static double HIVE_TILT_HEADING_DEG = 0;
    /**
     * Show each motor's and hub's current draw (amps). Each reading is a slow hub request, so only one
     * device is read per loop: every value refreshes once per (motors + hubs) loops.
     */
    public static boolean CURRENT_TELEMETRY = true;

    private static final String[] CURRENT_MOTORS = {
            "frontLeftMotor", "backLeftMotor", "frontRightMotor", "backRightMotor",
            "leftShooterMotor", "rightShooterMotor", "intakeMotor", "indexerMotor" };

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(DASHBOARD_INTERVAL_MS);   // default is 100 ms
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Read all motor/encoder values from each hub in one go per loop (much faster loops)
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
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

        // Same motor objects the subsystems use, only for reading current draw
        DcMotorEx[] currentMotors = new DcMotorEx[CURRENT_MOTORS.length];
        for (int i = 0; i < CURRENT_MOTORS.length; i++) {
            currentMotors[i] = hardwareMap.get(DcMotorEx.class, CURRENT_MOTORS[i]);
        }
        double[] motorAmps = new double[currentMotors.length];
        double[] hubAmps = new double[hubs.size()];
        int nextCurrentRead = 0;

        DataLogger log = LOG_TO_FILE ? new DataLogger("teleop",
                "targetRPM", "rawRPM", "filteredRPM", "shooterPower", "loadComp", "shotsFired",
                "readyToFeed", "volleyActive", "beamBroken", "magazineFull", "jamEjecting", "intakeVelocity",
                "sideServoDeg", "servoPowered", "batteryVoltage",
                "tagCount", "goalSeen", "goalRangeCm", "visionGoalDistanceCm", "aiming", "aimErrorDeg", "loopMs") : null;

        // Pick alliance (which goal to aim at) until START is pressed
        while (opModeInInit()) {
            if (gamepad1.x)      ThriftyCamera.ALLIANCE = ThriftyCamera.BLUE;
            else if (gamepad1.b) ThriftyCamera.ALLIANCE = ThriftyCamera.RED;
            camera.update();   // auto brightness tunes while waiting

            telemetry.addData("Status", "Initialized. Ready.");
            telemetry.addData("Camera", camera.isConnected() ? "connected" : "NOT FOUND (check config name)");
            telemetry.addData("Brightness", camera.getBrightnessStatus());
            telemetry.addData("Aiming at", ThriftyCamera.ALLIANCE + "   (X = blue, B = red)");
            telemetry.addData("Tags seen", camera.getTagIds());
            telemetry.update();
        }
        if (isStopRequested()) return;
        camera.lockBrightness();

        sideServos.goToB();   // servos don't move until first commanded

        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastA = false, lastB = false;
        boolean lastMagazineFull = false;
        boolean lastAligning = false;

        while (opModeIsActive()) {
            double batteryVoltage = battery.update();

            boolean autoAimShot = gamepad1.right_bumper;
            boolean shooting = autoAimShot || gamepad1.left_bumper;   // left bumper = same shot, no auto-aim

            // ================= CAMERA & AIM =================
            camera.update();
            double heading = drive.updateHeading();
            if (gamepad1.options) {
                drive.resetHeading();
                localizer.reset();   // the goal heading is in the IMU's frame, which just turned
            }
            localizer.update(heading);
            // Balls going through (feeder state is from last loop): stop turning so the shots aren't knocked off
            boolean firing = feeder.isVolleyActive() || (shooting && gamepad1.right_trigger_pressed);
            aim.update(autoAimShot, heading, firing);

            // ================= DRIVE =================
            // Auto-aim at the goal wins; otherwise D-pad up turns to HIVE_TILT_HEADING_DEG; otherwise the stick
            boolean aligning = gamepad1.dpad_up && !aim.isAiming();
            if (aligning && !lastAligning) drive.resetAlign();   // fresh controller memory on each press
            lastAligning = aligning;

            if (aim.isAiming()) {
                drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, aim.getTurn());
            } else if (aligning) {
                drive.alignWithHiveTilt(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        Math.toRadians(HIVE_TILT_HEADING_DEG));
            } else {
                drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // ================= SHOOTER =================
            if (shooting)                   shooter.spinUp();
            else if (feeder.isBackingOff()) shooter.setOpenLoopPower(Feeder.SHOOTER_BACKOFF_POWER);  // help push a ball back off the beam
            else if (feeder.isMagazineFull()) shooter.spinUp();   // get to speed before the shot (magazine state is from last loop)
            else                            shooter.stop();
            // Push harder while balls are going through (feeder state is from last loop, ~1 loop behind)
            shooter.setFeeding(firing);
            shooter.update(batteryVoltage);

            // ================= INTAKE & INDEXER =================
            feeder.update(
                    shooting, // Currently shooting Variable
                    shooting && shooter.isReadyToFeed() && aim.isReadyToShoot(), // Shooter revved up and aim settled (always true when not auto-aiming).
                    gamepad1.left_trigger_pressed, // Eject Keybind
                    gamepad1.right_trigger_pressed, // Manual Intake Keybind
                    gamepad1.y, // Run intake and index backwards.
                    false); // No magazine override in TeleOp

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
            telemetry.addData("cameraBrightness", camera.getBrightness());
            telemetry.addData("tagCount", camera.getTagCount());
            telemetry.addData("tagIds", camera.getTagIds());
            telemetry.addData("alliance", ThriftyCamera.ALLIANCE);
            telemetry.addData("goalSeen", camera.hasTarget() ? 1 : 0);
            telemetry.addData("goalRange (cm)", camera.getTargetRange());
            telemetry.addData("tagHeight (cm)", camera.getTagHeight());
            telemetry.addData("cameraTilt (deg)", camera.getCameraTiltDeg());
            telemetry.addData("visionGoalDistance (cm)", localizer.getVisionGoalDistance());
            telemetry.addData("aiming", aim.isAiming() ? 1 : 0);
            telemetry.addData("aimed", aim.isAimed() ? 1 : 0);
            telemetry.addData("aimReady", aim.isReadyToShoot() ? 1 : 0);
            telemetry.addData("aimHolding", aim.isHolding() ? 1 : 0);
            telemetry.addData("aimError (deg)", aim.getErrorDeg());

            // ================= CURRENT DRAW =================
            if (CURRENT_TELEMETRY) {
                // One device per loop (round-robin): motors first, then hubs
                if (nextCurrentRead < currentMotors.length) {
                    motorAmps[nextCurrentRead] = currentMotors[nextCurrentRead].getCurrent(CurrentUnit.AMPS);
                } else {
                    hubAmps[nextCurrentRead - currentMotors.length] =
                            hubs.get(nextCurrentRead - currentMotors.length).getCurrent(CurrentUnit.AMPS);
                }
                nextCurrentRead = (nextCurrentRead + 1) % (currentMotors.length + hubs.size());

                double motorTotal = 0, hubTotal = 0;
                for (int i = 0; i < currentMotors.length; i++) {
                    telemetry.addData(CURRENT_MOTORS[i] + " (A)", motorAmps[i]);
                    motorTotal += motorAmps[i];
                }
                for (int i = 0; i < hubs.size(); i++) {
                    String hubName = hubs.get(i).isParent() ? "controlHub" : "expansionHub" + i;
                    telemetry.addData(hubName + " (A)", hubAmps[i]);
                    hubTotal += hubAmps[i];
                }
                // Hub readings are the whole hub's draw, so what's left over is servos, camera and electronics
                telemetry.addData("totalCurrent (A)", hubTotal);
                telemetry.addData("otherCurrent (A)", hubTotal - motorTotal);
            }
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
                        camera.getTargetRange(), localizer.getVisionGoalDistance(),
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
