package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
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
 * The controller rumbles when the magazine becomes full.
 *
 * Every loop is also written to /sdcard/FIRST/logs/teleop_<date>_<time>.csv
 * (pull with: adb pull /sdcard/FIRST/logs).
 */
@Config
@Autonomous(name = "Auto", group = "Testing")
public class AutoTest extends LinearOpMode {

    /** How often (ms) the Dashboard graph gets a new point. Lower = smoother graph, more Wi-Fi traffic. */
    public static int DASHBOARD_INTERVAL_MS = 20;
    /** How long (ms) the controller rumbles when the magazine fills. 0 = off. */
    public static int FULL_RUMBLE_MS = 400;
    /** Save every loop to a CSV on the Control Hub. */
    public static boolean LOG_TO_FILE = false;
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


            telemetry.addData("Status", "Initialized. Ready.");
            telemetry.addData("Camera", camera.isConnected() ? "connected" : "NOT FOUND (check config name)");
            telemetry.addData("Aiming at", ThriftyCamera.ALLIANCE + "   (X = blue, B = red)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        sideServos.goToB();   // servos don't move until first commanded

        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastA = false, lastB = false;
        boolean lastMagazineFull = false;
        boolean lastAligning = false;

        while (opModeIsActive()) {

            ElapsedTime autoTimer = new ElapsedTime();
            double autoTimerAdjust = 1.0;

            //drive forwards
            while (autoTimer.seconds() <= autoTimerAdjust) {
            drive.drive(1,0,0);
            }


        drive.stop();
        shooter.stop();
        shooter.update(battery.get());
        feeder.stop();
        camera.close();
        if (log != null) log.close();
    }
}}
