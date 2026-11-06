package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.VoltageMonitor;

import java.util.ArrayList;
import java.util.List;

/**
 * Measures the shooter's ball-load settings: LOAD_VOLTS, SHOT_DROP_RPM and SHOT_RECOVER_RPM.
 * Run Shooter Characterization first (this relies on kS and kV being right).
 *
 * Each trial:
 *   1. LOAD   intake runs; put balls in, then press A (X = stop early and show results).
 *   2. SPIN   flywheel spins up to TARGET_RPM with load compensation OFF.
 *   3. HOLD   holds speed for HOLD_SEC with no balls: measures RPM noise and any kS/kV offset.
 *   4. FEED   feeds every ball exactly like TeleOp, recording for FEED_SEC.
 *
 * How LOAD_VOLTS is found: the motor needs  volts = kS + kV * RPM  to hold a speed.
 * While balls go through, anything applied above that is the balls' drag. That extra is
 * averaged from the start of feeding to when the speed has recovered after the last ball
 * (same speed at both ends, so flywheel acceleration cancels out). That average is the
 * constant push that would put the same energy back in over the same time.
 *
 * Results average over all trials. Press Y on the results screen to apply them to Shooter
 * for this session (they reset when the robot restarts; copy them into Shooter.java to keep them).
 * Every loop is also saved to /sdcard/FIRST/logs/loadcomp_<date>_<time>.csv.
 *
 * SAFETY: the flywheel runs at full shooting speed and fires balls. Aim it somewhere safe.
 */
@Config
@Autonomous(name = "Load Comp Characterization", group = "Testing")
public class LoadCompCharacterization extends LinearOpMode {

    public static int    TRIALS         = 3;
    public static double SPINUP_TIMEOUT = 5.0;   // give up if not at speed by then
    public static double HOLD_SEC       = 1.5;   // steady speed, no balls
    public static double FEED_SEC       = 3.0;   // long enough for every ball to go through and the speed to recover
    public static double MIN_DIP_RPM    = 15;    // smallest dip counted as a ball when analysing
    public static boolean LOG_TO_FILE   = true;

    /** One loop's reading. extraVolts = applied volts minus what kS + kV * RPM says is needed. */
    private static class Sample {
        final double t, rpm, extraVolts;
        Sample(double t, double rpm, double extraVolts) {
            this.t = t;
            this.rpm = rpm;
            this.extraVolts = extraVolts;
        }
    }

    /** What one trial found. */
    private static class Trial {
        boolean valid;
        String note = "";
        int balls;
        double noiseRpm, offsetVolts;
        double minDipRpm = Double.MAX_VALUE, maxDipRpm;
        double windowSec, loadVolts;
    }

    private Shooter shooter;
    private Feeder feeder;
    private VoltageMonitor battery;
    private VoltageSensor voltageSensor;
    private DataLogger log;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Same subsystems and setup as TeleOp, so the balls go through exactly the same way
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        battery = new VoltageMonitor(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        log = LOG_TO_FILE ? new DataLogger("loadcomp",
                "trial", "phase", "rpm", "power", "batteryVoltage", "appliedVolts", "extraVolts") : null;

        telemetry.addLine("Run Shooter Characterization first (needs kS and kV).");
        telemetry.addLine("Flywheel will spin up and FIRE balls. Aim it somewhere safe.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        List<Trial> trials = new ArrayList<>();
        for (int i = 1; i <= TRIALS && opModeIsActive(); i++) {
            if (!waitForLoad(i)) break;   // X pressed: finish early
            Trial trial = runTrial(i);
            if (trial == null) break;     // stopped mid-trial
            trials.add(trial);
        }

        shooter.stop();
        shooter.update(battery.get());
        feeder.stop();
        if (log != null) log.close();

        showResults(trials);
    }

    /** Runs the intake so balls can be loaded. Returns false if X was pressed (finish now). */
    private boolean waitForLoad(int trialNumber) {
        while (opModeIsActive() && (gamepad1.a || gamepad1.x)) idleLoop();   // wait for release from last time
        while (opModeIsActive()) {
            if (gamepad1.a) return true;
            if (gamepad1.x) return false;
            idleLoop();
            telemetry.addData("Trial", "%d of %d", trialNumber, TRIALS);
            telemetry.addLine("Load balls (intake is running), then press A.");
            telemetry.addLine("X = stop here and show results.");
            telemetry.addData("magazineFull", feeder.isMagazineFull());
            telemetry.update();
        }
        return false;
    }

    /** Shooter off, intake loading, same as TeleOp when no buttons are held. */
    private void idleLoop() {
        if (feeder.isBackingOff()) shooter.setOpenLoopPower(Feeder.SHOOTER_BACKOFF_POWER);
        else                       shooter.stop();
        shooter.setFeeding(false);
        shooter.update(battery.update());
        feeder.update(false, false, false, false, false);
    }

    /** Spin up, hold, feed. Returns null if the OpMode was stopped. */
    private Trial runTrial(int trialNumber) {
        Trial trial = new Trial();

        // ---- SPIN UP (shoot held, not ready yet: indexer holds the balls back like TeleOp) ----
        clock.reset();
        while (opModeIsActive() && !shooter.isReadyToFeed()) {
            step(trialNumber, -1, false);
            showLive(trialNumber, "spinning up");
            if (clock.seconds() > SPINUP_TIMEOUT) {
                trial.note = "never reached TARGET_RPM (check kS/kV and battery)";
                return trial;
            }
        }

        // ---- HOLD: no balls ----
        List<Sample> hold = new ArrayList<>();
        clock.reset();
        while (opModeIsActive() && clock.seconds() < HOLD_SEC) {
            hold.add(step(trialNumber, 0, false));
            showLive(trialNumber, "holding (no balls)");
        }

        // ---- FEED: every ball, like TeleOp once the shooter is ready ----
        List<Sample> feed = new ArrayList<>();
        clock.reset();
        while (opModeIsActive() && clock.seconds() < FEED_SEC) {
            feed.add(step(trialNumber, 1, true));
            showLive(trialNumber, "FEEDING");
        }
        if (!opModeIsActive()) return null;

        analyse(trial, hold, feed);
        return trial;
    }

    /**
     * One control loop with load compensation OFF (setFeeding false), so we see the raw effect of the balls.
     * @param phase  logged: -1 spin-up, 0 hold, 1 feed
     */
    private Sample step(int trialNumber, int phase, boolean feedBalls) {
        double batteryNow = voltageSensor.getVoltage();   // read every loop: it sags while the wheel works

        shooter.spinUp();
        shooter.setFeeding(false);
        shooter.update(battery.update());
        feeder.update(true, feedBalls, false, false, false);

        double rpm = shooter.getRpm();
        double applied = shooter.getPower() * batteryNow;
        double extra = applied - (Shooter.kS + Shooter.kV * rpm);

        if (log != null) log.log(trialNumber, phase, rpm, shooter.getPower(), batteryNow, applied, extra);
        return new Sample(clock.seconds(), rpm, extra);
    }

    private void analyse(Trial trial, List<Sample> hold, List<Sample> feed) {
        double target = Shooter.TARGET_RPM;

        // Hold: how far below target the RPM wanders on its own, and the kS/kV offset
        double offsetSum = 0;
        for (Sample s : hold) {
            trial.noiseRpm = Math.max(trial.noiseRpm, target - s.rpm);
            offsetSum += s.extraVolts;
        }
        trial.offsetVolts = hold.isEmpty() ? 0 : offsetSum / hold.size();

        // Feed: find each ball's dip (drops clearly below the noise, then recovers)
        double dropAt = Math.max(MIN_DIP_RPM, 2 * trial.noiseRpm);
        double recoverAt = Math.max(5, trial.noiseRpm);
        boolean inDip = false;
        double dipLowest = 0;
        int lastRecovered = -1;
        for (int i = 0; i < feed.size(); i++) {
            double rpm = feed.get(i).rpm;
            if (!inDip) {
                if (rpm < target - dropAt) {
                    inDip = true;
                    dipLowest = rpm;
                }
            } else {
                dipLowest = Math.min(dipLowest, rpm);
                if (rpm > target - recoverAt) {
                    inDip = false;
                    double depth = target - dipLowest;
                    trial.balls++;
                    trial.minDipRpm = Math.min(trial.minDipRpm, depth);
                    trial.maxDipRpm = Math.max(trial.maxDipRpm, depth);
                    lastRecovered = i;
                }
            }
        }

        if (trial.balls == 0) {
            trial.note = "no balls seen (were any loaded?)";
            return;
        }
        int end = lastRecovered;
        if (inDip) {
            trial.note = "speed hadn't recovered after the last ball: raise FEED_SEC";
            end = feed.size() - 1;
        }

        // Time-weighted average of the extra volts from feed start to the last recovery
        double voltSeconds = 0;
        for (int i = 1; i <= end; i++) {
            double dt = feed.get(i).t - feed.get(i - 1).t;
            voltSeconds += (feed.get(i).extraVolts - trial.offsetVolts) * dt;
        }
        trial.windowSec = feed.get(end).t - feed.get(0).t;
        trial.loadVolts = trial.windowSec > 0 ? voltSeconds / trial.windowSec : 0;
        trial.valid = true;
    }

    private void showResults(List<Trial> trials) {
        int valid = 0;
        double loadSum = 0, noise = 0, minDip = Double.MAX_VALUE, ballSum = 0;
        for (Trial t : trials) {
            if (!t.valid) continue;
            valid++;
            loadSum += t.loadVolts;
            noise = Math.max(noise, t.noiseRpm);
            minDip = Math.min(minDip, t.minDipRpm);
            ballSum += t.balls;
        }

        double loadVolts = 0, dropRpm = 0, recoverRpm = 0;
        if (valid > 0) {
            loadVolts = loadSum / valid;
            dropRpm = Math.round((noise + minDip) / 2);   // halfway between wobble and the smallest ball dip
            recoverRpm = Math.round((noise + dropRpm) / 2);
        }

        boolean applied = false;
        while (opModeIsActive()) {
            if (valid > 0 && gamepad1.y && !applied) {
                Shooter.LOAD_VOLTS = loadVolts;
                Shooter.SHOT_DROP_RPM = dropRpm;
                Shooter.SHOT_RECOVER_RPM = recoverRpm;
                Shooter.BALLS_PER_VOLLEY = (int) Math.round(ballSum / valid);
                applied = true;
            }

            telemetry.addLine("=== RESULTS ===");
            if (valid > 0) {
                telemetry.addData("LOAD_VOLTS", "%.3f   (now %.3f)", loadVolts, Shooter.LOAD_VOLTS);
                telemetry.addData("SHOT_DROP_RPM", "%.0f   (now %.0f)", dropRpm, Shooter.SHOT_DROP_RPM);
                telemetry.addData("SHOT_RECOVER_RPM", "%.0f   (now %.0f)", recoverRpm, Shooter.SHOT_RECOVER_RPM);
                telemetry.addData("BALLS_PER_VOLLEY", "%.1f avg seen   (now %d)", ballSum / valid, Shooter.BALLS_PER_VOLLEY);
                telemetry.addData("RPM noise / smallest dip", "%.0f / %.0f", noise, minDip);
                if (minDip < 2 * noise) {
                    telemetry.addLine("WARNING: ball dips barely stand out from the noise; shot counting may be unreliable.");
                }
                telemetry.addLine(applied ? "Applied to Shooter for this session. Copy into Shooter.java to keep."
                                          : "Y = apply to Shooter for this session.");
            } else {
                telemetry.addLine("No valid trials.");
            }

            telemetry.addLine("trial | balls | load V | dips (min-max) | noise | offset V | window s");
            for (int i = 0; i < trials.size(); i++) {
                Trial t = trials.get(i);
                if (t.valid) {
                    telemetry.addLine(String.format("%d | %d | %.3f | %.0f-%.0f | %.0f | %.3f | %.2f %s",
                            i + 1, t.balls, t.loadVolts, t.minDipRpm, t.maxDipRpm,
                            t.noiseRpm, t.offsetVolts, t.windowSec, t.note));
                } else {
                    telemetry.addLine((i + 1) + " | invalid: " + t.note);
                }
            }
            telemetry.update();
        }
    }

    private void showLive(int trialNumber, String phase) {
        telemetry.addData("Trial", "%d of %d   (%s)", trialNumber, TRIALS, phase);
        telemetry.addData("targetRPM", Shooter.TARGET_RPM);
        telemetry.addData("rpm", shooter.getRpm());
        telemetry.addData("shooterPower", shooter.getPower());
        telemetry.update();
    }
}
