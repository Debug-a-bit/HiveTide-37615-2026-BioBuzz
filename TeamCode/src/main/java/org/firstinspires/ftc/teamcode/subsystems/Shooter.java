package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.LowPassFilter;

/**
 * Two goBILDA 5000 series motors (28 ticks/rev, no gearbox) on one flywheel axle.
 *
 * Speed control works in VOLTS, then divides by battery voltage so it behaves
 * the same on a full or a tired battery:
 *
 *     volts = kS + kV * targetRPM + kP * (targetRPM - measuredRPM)
 *     power = volts / batteryVoltage        (clamped 0..1, never brakes)
 *
 * While balls are being fed (see setFeeding), extra volts are added up front
 * (LOAD_VOLTS) and a stiffer kP is used, so the flywheel is already pushing
 * harder when each ball hits it instead of waiting for the speed to drop.
 * Each ball shows up as an RPM dip; once BALLS_PER_VOLLEY dips have recovered,
 * the extra push is removed so an empty flywheel doesn't overspeed.
 *
 * Find kS and kV with the "Shooter Characterization" OpMode.
 */
@Config
public class Shooter {

    public static final double TICKS_PER_REV = 28;

    // ---- Controller (live-tunable in FTC Dashboard) ----
    public static double kS = 0.7440;      // volts needed just to start turning
    public static double kV = 0.002092;   // volts per RPM, measured automatically
    public static double kP = 0.05;       // volts per RPM of error

    public static double TARGET_RPM = 3050;
    public static double FILTER_SEC = 0.005;   // speed smoothing; lower = less delay

    // ---- "Ready to feed" check ----
    public static double READY_TIME_SEC     = 0.4; // must hold speed this long before feeding
    public static double DROP_TOLERANCE_RPM = 50;   // dropping this far below target = not ready

    // ---- Ball load compensation (only while feeding) ----
    public static int    BALLS_PER_VOLLEY = 4;
    public static double LOAD_VOLTS       = 0.39;   // extra volts to carry the balls' drag
    public static double kP_FEEDING       = 0.06;  // stiffer speed correction while balls go through
    public static double SHOT_DROP_RPM    = 25;   // a dip this far below target = a ball is in the wheel
    public static double SHOT_RECOVER_RPM = 10;    // back within this of target = that ball has left

    private enum Mode { OFF, VELOCITY, OPEN_LOOP }

    private final DcMotorEx right, left;
    private final LowPassFilter rpmFilter = new LowPassFilter(FILTER_SEC);
    private final ElapsedTime readyTimer = new ElapsedTime();

    private Mode mode = Mode.OFF;
    private double openLoopPower = 0;
    private double power = 0;
    private double rawRpm = 0;
    private double rpm = 0;
    private boolean atSpeed = false;

    private boolean feeding = false;
    private boolean inShotDip = false;
    private int shotsFired = 0;

    public Shooter(HardwareMap hardwareMap) {
        right = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        left  = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        // The motors face each other on the same axle, so one is reversed
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{ right, left }) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast, don't brake
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            // our controller, not the hub's
        }
    }

    // ---- Commands: call one of these each loop, then update() ----

    /** Hold TARGET_RPM using the speed controller. */
    public void spinUp() {
        mode = Mode.VELOCITY;
    }

    /** Run both motors at a fixed power (-1..1), e.g. for backing a ball off or testing. */
    public void setOpenLoopPower(double power) {
        mode = Mode.OPEN_LOOP;
        openLoopPower = power;
    }

    public void stop() {
        mode = Mode.OFF;
    }

    /** Tell the shooter whether balls are being fed into it right now. Call each loop. */
    public void setFeeding(boolean feeding) {
        if (feeding && !this.feeding) {   // new volley: start counting from zero
            shotsFired = 0;
            inShotDip = false;
        }
        this.feeding = feeding;
    }

    /** Reads the encoders, runs the controller and sets motor power. Call once per loop. */
    public void update(double batteryVoltage) {
        // Average both encoders (quieter), then smooth
        rawRpm = ticksPerSecToRpm((right.getVelocity() + left.getVelocity()) / 2.0);
        rpmFilter.setTimeConstant(FILTER_SEC);
        rpm = rpmFilter.update(rawRpm);

        countShots();
        boolean loaded = isLoadCompensating();

        switch (mode) {
            case VELOCITY:
                double volts = kS * Math.signum(TARGET_RPM)
                             + kV * TARGET_RPM
                             + (loaded ? LOAD_VOLTS : 0)
                             + (loaded ? kP_FEEDING : kP) * (TARGET_RPM - rpm);
                power = Range.clip(volts / batteryVoltage, 0.0, 1.0);   // never negative: coast down
                break;
            case OPEN_LOOP:
                power = Range.clip(openLoopPower, -1.0, 1.0);
                break;
            default:
                power = 0;
        }
        right.setPower(power);
        left.setPower(power);

        updateReady();
    }

    /** Counts a ball each time the speed dips from a shot and then recovers. */
    private void countShots() {
        if (!feeding || mode != Mode.VELOCITY) {
            inShotDip = false;
        } else if (!inShotDip) {
            if (rpm < TARGET_RPM - SHOT_DROP_RPM) inShotDip = true;
        } else if (rpm > TARGET_RPM - SHOT_RECOVER_RPM) {
            inShotDip = false;
            shotsFired++;
        }
    }

    /** True while the extra push for the balls is being applied. */
    public boolean isLoadCompensating() {
        return feeding && mode == Mode.VELOCITY && shotsFired < BALLS_PER_VOLLEY;
    }

    public int getShotsFired() { return shotsFired; }

    /** Latches "at speed" once target is reached; drops it only on a real speed drop. */
    private void updateReady() {
        if (mode != Mode.VELOCITY) {
            atSpeed = false;
        } else if (!atSpeed) {
            if (rpm >= TARGET_RPM) {
                atSpeed = true;
                readyTimer.reset();
            }
        } else if (rpm < TARGET_RPM - DROP_TOLERANCE_RPM) {
            atSpeed = false;
        }
    }

    /** True once the flywheel has held target speed for READY_TIME_SEC. */
    public boolean isReadyToFeed() {
        return atSpeed && readyTimer.seconds() >= READY_TIME_SEC;
    }

    public double getRpm()       { return rpm; }
    public double getRawRpm()    { return rawRpm; }
    public double getPower()     { return power; }
    public double getTargetRpm() { return mode == Mode.VELOCITY ? TARGET_RPM : 0; }

    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_REV * 60.0;
    }

    public static double rpmToTicksPerSec(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }
}
