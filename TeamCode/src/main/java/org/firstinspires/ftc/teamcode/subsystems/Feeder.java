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
 * Magazine logic:
 *   - Beam broken while not shooting  -> magazine is full. Everything backs off
 *     (intake, indexer, and shooter reverse) until the ball clears the beam.
 *   - Magazine full                   -> intake and indexer stop and hold.
 *   - Shooter ready / reject pressed  -> magazine no longer full; intake runs again.
 */
@Config
public class Feeder {

    public static double INTAKE_RUNNING_POWER = 1.0;
    public static double INTAKE_IDLE_POWER    = 0.0;
    public static double INTAKE_REVERSE_POWER = -1.0;

    public static double INDEXER_RUNNING_POWER = 0.7;
    public static double INDEXER_IDLE_POWER    = -1.0;
    public static double INDEXER_REVERSE_POWER = -1.0;

    /** Shooter power while backing a ball off the beam break (set 0 to disable). */
    public static double SHOOTER_BACKOFF_POWER = -1.0;

    private final DcMotorEx intake, indexer;
    private final DigitalChannel beamBreak;

    private boolean magazineFull = false;
    private boolean beamBroken = false;
    private boolean backingOff = false;

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
     * @param shooterReady   shooter has held target speed long enough to auto-feed
     * @param reject         driver wants to spit balls out of the intake
     * @param manualFeed     driver override: push indexer toward the shooter
     * @param manualReverse  driver override: run indexer backwards
     */
    public void update(boolean shooting, boolean shooterReady, boolean reject,
                       boolean manualFeed, boolean manualReverse) {
        beamBroken = !beamBreak.getState();   // sensor reads false when the beam is blocked
        backingOff = beamBroken && !shooting;

        if (backingOff) {
            magazineFull = true;
        } else if (reject || shooterReady) {
            magazineFull = false;
        }

        // ---- Intake ----
        double intakePower;
        if (manualFeed) intakePower = INTAKE_RUNNING_POWER;
        else if (backingOff || reject) intakePower = INTAKE_REVERSE_POWER;
        else if (!magazineFull)   intakePower = INTAKE_RUNNING_POWER;
        else                      intakePower = INTAKE_IDLE_POWER;
        intake.setPower(intakePower);

        // ---- Indexer (manual buttons always win) ----
        double indexerPower;
        if (manualFeed)          indexerPower = INDEXER_RUNNING_POWER;
        else if (manualReverse)  indexerPower = INDEXER_REVERSE_POWER;
        else if (backingOff)     indexerPower = INDEXER_REVERSE_POWER;
        else if (shooterReady)   indexerPower = INDEXER_RUNNING_POWER;   // auto-feed
        else if (magazineFull)   indexerPower = 0;
        else                     indexerPower = INDEXER_IDLE_POWER;
        indexer.setPower(indexerPower);
    }

    /** True while a ball is being pushed back off the beam break. */
    public boolean isBackingOff()   { return backingOff; }
    public boolean isMagazineFull() { return magazineFull; }
    public boolean isBeamBroken()   { return beamBroken; }

    public void stop() {
        intake.setPower(0);
        indexer.setPower(0);
    }
}
