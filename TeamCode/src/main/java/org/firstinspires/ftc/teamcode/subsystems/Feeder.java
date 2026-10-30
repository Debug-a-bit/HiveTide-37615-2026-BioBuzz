package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake + indexer + the beam break that detects a full magazine.
 *
 * Manual (triggers held):
 *   - Manual feed     -> intake and indexer run forward.
 *   - Manual reverse  -> intake and indexer run backward.
 *   The automatic back-off is paused so it can't fight the driver,
 *   but the magazine is still tracked so auto picks up correctly on release.
 *
 * Automatic (no trigger held):
 *   - Beam broken while not shooting  -> magazine is full. Everything backs off
 *     (intake, indexer, and shooter reverse) until the ball clears the beam.
 *   - Magazine full                   -> intake and indexer stop and hold.
 *   - Shooter first ready             -> volley starts: indexer keeps feeding every
 *     ball (no re-waiting for speed between shots) until shoot is released.
 *   - Balls leave / reject pressed    -> magazine no longer full; intake runs again.
 */
@Config
public class Feeder {

    /** False = indexer never feeds on its own; use the manual triggers to shoot. */
    public static boolean AUTO_FEED_ENABLED   = true;
    /** False = intake never runs on its own; use the manual triggers (reject and back-off still work). */
    public static boolean AUTO_INTAKE_ENABLED = true;

    public static double INTAKE_RUNNING_POWER = 1.0;
    public static double INTAKE_IDLE_POWER    = 0.0;
    public static double INTAKE_REVERSE_POWER = -1.0;

    public static double INDEXER_RUNNING_POWER = 0.7;
    public static double INDEXER_IDLE_POWER    = -1.0;
    public static double INDEXER_REVERSE_POWER = -1.0;

    /** Shooter power while backing a ball off the beam break (set 0 to disable). */
    public static double SHOOTER_BACKOFF_POWER = 0.0;

    private final DcMotorEx intake, indexer;
    private final DigitalChannel beamBreak;

    private boolean magazineFull = false;
    private boolean beamBroken = false;
    private boolean backingOff = false;
    private boolean manual = false;
    private boolean volley = false;

    public Feeder(HardwareMap hardwareMap) {
        intake    = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        indexer   = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        indexer.setDirection(DcMotorSimple.Direction.REVERSE);   // so + feeds toward the shooter

        for (DcMotorEx motor : new DcMotorEx[]{ intake, indexer }) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Call once per loop.
     *
     * @param shooting       driver is holding the shoot button
     * @param shooterReady   shooter has held target speed long enough to start auto-feeding
     * @param reject         driver wants to spit balls out of the intake
     * @param manualFeed     manual: run intake + indexer toward the shooter
     * @param manualReverse  manual: run intake + indexer backwards
     */
    public void update(boolean shooting, boolean shooterReady, boolean reject,
                       boolean manualFeed, boolean manualReverse) {
        beamBroken = !beamBreak.getState();   // sensor reads false when the beam is blocked
        manual = manualFeed || manualReverse;

        // Once the shooter is first ready, keep feeding until shoot is released,
        // so the speed dip from each shot doesn't pause the volley.
        volley = AUTO_FEED_ENABLED && shooting && (volley || shooterReady);

        // ---- Track the magazine (always, even in manual) ----
        if (beamBroken && !shooting) {
            magazineFull = true;                                    // a ball has reached the top
        } else if (reject || volley || (shooting && manualFeed)) {
            magazineFull = false;                                   // balls are leaving
        }

        // Automatic back-off only when the driver isn't controlling the feeder
        backingOff = beamBroken && !shooting && !manual;

        // ---- Intake ----
        double intakePower;
        if (manualFeed)                intakePower = INTAKE_RUNNING_POWER;
        else if (manualReverse)        intakePower = INTAKE_REVERSE_POWER;
        else if (backingOff || reject) intakePower = INTAKE_REVERSE_POWER;
        else if (AUTO_INTAKE_ENABLED && !magazineFull) intakePower = INTAKE_RUNNING_POWER;
        else                           intakePower = INTAKE_IDLE_POWER;
        intake.setPower(intakePower);

        // ---- Indexer ----
        double indexerPower;
        if (manualFeed)          indexerPower = INDEXER_RUNNING_POWER;
        else if (manualReverse)  indexerPower = INDEXER_REVERSE_POWER;
        else if (backingOff)     indexerPower = INDEXER_REVERSE_POWER;
        else if (volley)         indexerPower = INDEXER_RUNNING_POWER;   // auto-feed every ball
        else if (magazineFull || !AUTO_FEED_ENABLED)   indexerPower = 0;                       // hold
        else                     indexerPower = INDEXER_IDLE_POWER;
        indexer.setPower(indexerPower);
    }

    /** True while a ball is being pushed back off the beam break (never during manual). */
    public boolean isBackingOff()   { return backingOff; }
    public boolean isMagazineFull() { return magazineFull; }
    public boolean isBeamBroken()   { return beamBroken; }
    public boolean isManual()       { return manual; }
    public boolean isVolleyActive() { return volley; }

    public void stop() {
        intake.setPower(0);
        indexer.setPower(0);
    }
}