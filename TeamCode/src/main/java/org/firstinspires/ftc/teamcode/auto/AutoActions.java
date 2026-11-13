package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Building blocks for autonomous OpModes: timed moves, turns to a heading, aiming and shooting.
 * Every move, turn and pause has an intake on/off option. Create one in runOpMode(), call markStart() right after START, then
 * chain the actions. Every action ends early if the OpMode is stopped.
 *
 * Moves are field-centric and relative to the heading at markStart(): FORWARD etc. stay the same
 * field direction whichever way the robot is facing.
 *
 * The shooter spins up as soon as the magazine is full (and always while aiming and shooting), so each
 * volley starts already at speed. It coasts while the magazine is still filling.
 *
 * Every loop is written to /sdcard/FIRST/logs/auto_<date>_<time>.csv (pull with: adb pull /sdcard/FIRST/logs).
 * The "step" column counts actions in the order they ran (1 = first move/turn/aim/shoot).
 *
 * Timed moves scale their power by NOMINAL_VOLTS / battery voltage, so a move covers the same distance
 * on a full or a tired battery.
 */
@Config
public class AutoActions {

    /** False = timed moves use their power as given, whatever the battery voltage. */
    public static boolean VOLTAGE_COMPENSATION = true;
    /** Battery voltage (under load, as "batteryVoltage" shows) the move distances were tuned at. */
    public static double NOMINAL_VOLTS = 12.5;

    /** Field directions, relative to how the robot sat at markStart(). */
    public enum Direction {
        FORWARD(-1, 0), BACKWARD(1, 0), LEFT(0, -1), RIGHT(0, 1);

        final double x, y;   // drive.drive() strafe and forward inputs at full speed

        Direction(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Direction opposite() {
            switch (this) {
                case FORWARD:  return BACKWARD;
                case BACKWARD: return FORWARD;
                case LEFT:     return RIGHT;
                default:       return LEFT;
            }
        }
    }

    public final FieldCentricDrive drive;
    public final Shooter shooter;
    public final Feeder feeder;
    public final SideServos sideServos;
    public final VoltageMonitor battery;
    public final ThriftyCamera camera;
    public final Localizer localizer;
    public final AutoAim aim;

    private final LinearOpMode opMode;
    private double startHeadingRad = 0;

    private final DataLogger log;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private int step = 0;

    /** Sets up every subsystem. Call from runOpMode(), before waiting for START. */
    public AutoActions(LinearOpMode opMode) {
        this.opMode = opMode;

        // Read all motor/encoder values from each hub in one go per loop (much faster loops)
        for (LynxModule hub : opMode.hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive      = new FieldCentricDrive(opMode.hardwareMap);
        shooter    = new Shooter(opMode.hardwareMap);
        feeder     = new Feeder(opMode.hardwareMap);
        sideServos = new SideServos(opMode.hardwareMap);
        battery    = new VoltageMonitor(opMode.hardwareMap);
        camera     = new ThriftyCamera(opMode.hardwareMap);
        localizer  = new Localizer(camera);
        aim        = new AutoAim(localizer);

        log = new DataLogger("auto",
                "step", "headingDeg", "targetRPM", "rawRPM", "filteredRPM", "shooterPower", "loadComp", "shotsFired",
                "readyToFeed", "volleyActive", "beamBroken", "magazineFull", "backingOff", "jamEjecting",
                "intakeVelocity", "batteryVoltage", "goalSeen", "goalRangeCm", "aiming", "aimErrorDeg",
                "aimReady", "loopMs");
    }

    /** Call right after START: zeroes the heading so directions and turns are relative to how the robot starts. */
    public void markStart() {
        drive.resetHeading();
        startHeadingRad = Math.toRadians(drive.updateHeading());
    }

    // ================= MOVES =================
    // intake: INTAKE_ON = intake runs (until the magazine is full) and is left running at the end, so
    // moves can be chained; INTAKE_OFF = intake off.

    public static final boolean INTAKE_ON = true;
    public static final boolean INTAKE_OFF = false;

    private double normalSpeed = 0.6;

    /** The speed move() uses when none is given. Distances are measured as seconds at this speed. */
    public void setNormalSpeed(double speed) {
        normalSpeed = speed;
    }

    /** Drives a distance (seconds at normal speed) in a direction at normal speed, then stops. */
    public void move(Direction direction, double distance, boolean intake) {
        move(direction, distance, intake, normalSpeed);
    }

    /**
     * Drives a distance (seconds at normal speed) at another speed, then stops. The time is scaled so
     * it covers the same distance: half the speed takes twice as long.
     */
    public void move(Direction direction, double distance, boolean intake, double speed) {
        double seconds = distance * normalSpeed / speed;
        timedDrive(direction.x * speed, direction.y * speed, 0, seconds, intake);
    }

    /** Turns on the spot for a set time (power + = clockwise), then stops. */
    public void turnFor(double seconds, double power, boolean intake) {
        timedDrive(0, 0, power, seconds, intake);
    }

    /** Waits with the drive stopped. */
    public void pause(double seconds, boolean intake) {
        timedDrive(0, 0, 0, seconds, intake);
    }

    /**
     * Shuffles one way then back, stopping for pauseSeconds at each turnaround. Each push covers
     * eachWayDistance (seconds at normal speed) at speed, so it ends where it started.
     */
    public void wiggle(Direction first, int times, double eachWayDistance, double pauseSeconds, boolean intake, double speed) {
        for (int i = 0; i < times; i++) {
            move(first, eachWayDistance, intake, speed);
            pause(pauseSeconds, intake);
            move(first.opposite(), eachWayDistance, intake, speed);
            pause(pauseSeconds, intake);
        }
    }

    public void stopIntake() {
        feeder.stop();
    }

    /** Side servos up (angle B). Starts the move and carries straight on; the servos finish on their own. */
    public void servoUp() {
        sideServos.goToB();
    }

    /** Side servos down (angle A). Starts the move and carries straight on; the servos finish on their own. */
    public void servoDown() {
        sideServos.goToA();
    }

    private void timedDrive(double x, double y, double rx, double seconds, boolean intake) {
        step++;
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() <= seconds) {
            drive.updateHeading();
            double scale = voltageScale();
            drive.drive(x * scale, y * scale, rx * scale);
            updateIntake(intake);
            updateShooter(feeder.isMagazineFull());
            logLoop();
        }
        drive.drive(0, 0, 0);
    }

    /** Power multiplier that makes the drive behave as it would at NOMINAL_VOLTS. */
    private double voltageScale() {
        double volts = battery.update();
        if (!VOLTAGE_COMPENSATION || volts <= 0) return 1.0;
        return NOMINAL_VOLTS / volts;
    }

    /**
     * One loop of the intake: on = runs until the beam break says the magazine is full, then backs off and holds.
     * Off still finishes a back-off and keeps holding a full magazine (the intake and indexer don't run then),
     * so a ball left on the beam isn't pressed against the spinning shooter.
     */
    private void updateIntake(boolean on) {
        if (on || feeder.isMagazineFull()) feeder.update(false, false, false, false, false, false);
        else                               feeder.stop();
    }

    /** One loop of the shooter: spin = hold TARGET_RPM (extra push while a volley goes through), otherwise coast. */
    private void updateShooter(boolean spin) {
        if (spin && !feeder.isBackingOff()) shooter.spinUp();   // not while a ball is pulled back off it
        else                                shooter.stop();
        shooter.setFeeding(feeder.isVolleyActive());
        shooter.update(battery.update());
    }

    // ================= TURNS =================

    /**
     * Turns on the spot to a heading measured from the start (degrees, + = counter-clockwise).
     * Stops once within FieldCentricDrive.ALIGN_TOLERANCE_DEG, or gives up after timeoutSeconds.
     */
    public void turnTo(double degreesFromStart, double timeoutSeconds, boolean intake) {
        step++;
        double targetRad = startHeadingRad + Math.toRadians(degreesFromStart);
        drive.resetAlign();
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() <= timeoutSeconds) {
            drive.updateHeading();
            drive.alignWithHiveTilt(0, 0, targetRad);
            updateIntake(intake);
            updateShooter(feeder.isMagazineFull());
            logLoop();
            if (Math.abs(drive.getAlignErrorDegrees()) < FieldCentricDrive.ALIGN_TOLERANCE_DEG) break;
        }
        drive.drive(0, 0, 0);
    }

    // ================= AIM AND SHOOT =================

    /** Turns toward the goal for a set time (only while the camera can see it), then stops turning. */
    public void alignToGoal(double seconds) {
        step++;
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() <= seconds) {
            updateAim();
            updateShooter(true);
            logLoop();
            opMode.telemetry.update();
        }
        drive.drive(0, 0, 0);
    }

    /**
     * Fires a full volley (Shooter.BALLS_PER_VOLLEY), still aiming until feeding starts, then stops
     * the feeder. The shooter keeps spinning until the next action sees the magazine isn't full.
     * Gives up after timeoutSeconds.
     */
    public void shoot(double timeoutSeconds) {
        step++;
        shooter.resetShotCount();
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && shooter.getShotsFired() < Shooter.BALLS_PER_VOLLEY && timer.seconds() <= timeoutSeconds) {
            updateAim();
            updateShooter(true);
            feeder.update(
                    true,                                            // shooting
                    shooter.isReadyToFeed() && aim.isReadyToShoot(), // at speed and aim settled
                    false, false, false,                             // no eject / manual feed / manual reverse
                    !feeder.isVolleyActive());                       // preloaded: full until feeding starts, then the intake pushes balls up
            logLoop();
        }

        // Release "shoot" so the next volley starts fresh
        drive.drive(0, 0, 0);
        aim.update(false, drive.getHeadingDegrees(), false);
        shooter.setFeeding(false);
        shooter.update(battery.get());
        // Treat the magazine as empty so the next pickup runs the intake, even if the volley timed out
        // (the beam break marks it full again straight away if a ball is still at the top)
        feeder.clearMagazine();
        feeder.update(false, false, false, false, false, false);
        feeder.stop();
    }

    /** One loop of auto-aim: reads the camera and IMU, then turns toward the goal if it's in view. */
    private void updateAim() {
        camera.update();
        double heading = drive.updateHeading();
        localizer.update(heading);
        aim.update(true, heading, feeder.isVolleyActive());
        drive.drive(0, 0, aim.getTurn());
    }

    /** Stops every motor and closes the camera. Call at the end of the OpMode. */
    public void stopAll() {
        drive.stop();
        shooter.stop();
        shooter.update(battery.get());
        feeder.stop();
        camera.close();
        log.close();
    }

    /** Writes this loop to the log file. */
    private void logLoop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        log.log(step, drive.getHeadingDegrees(),
                shooter.getTargetRpm(), shooter.getRawRpm(), shooter.getRpm(), shooter.getPower(),
                shooter.isLoadCompensating() ? 1 : 0, shooter.getShotsFired(), shooter.isReadyToFeed() ? 1 : 0,
                feeder.isVolleyActive() ? 1 : 0, feeder.isBeamBroken() ? 1 : 0, feeder.isMagazineFull() ? 1 : 0,
                feeder.isBackingOff() ? 1 : 0, feeder.isJamEjecting() ? 1 : 0, feeder.getIntakeVelocity(),
                battery.get(), camera.hasTarget() ? 1 : 0, camera.getTargetRange(),
                aim.isAiming() ? 1 : 0, aim.getErrorDeg(), aim.isReadyToShoot() ? 1 : 0, loopMs);
    }
}
