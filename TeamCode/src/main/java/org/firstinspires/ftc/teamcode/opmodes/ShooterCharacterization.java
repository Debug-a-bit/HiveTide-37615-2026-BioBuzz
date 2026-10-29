package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the shooter's kS and kV by stepping through fixed powers,
 * waiting for the flywheel to settle, and recording
 *     appliedVolts = power x batteryVoltage   vs   RPM.
 * Then fits a straight line:  volts = kS + kV * RPM.
 * Copy the results into Shooter.kS and Shooter.kV.
 *
 * Run it on a full battery, then on a part-drained one. If kS/kV come out
 * close both times, the voltage compensation is working.
 *
 * SAFETY: the flywheel goes up to full speed. Keep hands and balls clear.
 */
@Config
@Autonomous(name = "Shooter Characterization", group = "Testing")
public class ShooterCharacterization extends LinearOpMode {

    public static double START_POWER   = 0.2;
    public static double END_POWER     = 1.0;
    public static double POWER_STEP    = 0.1;
    public static double SETTLE_SEC    = 2.0;   // let the flywheel reach steady speed
    public static double SAMPLE_SEC    = 1.0;   // then average readings over this long
    public static double MIN_VALID_RPM = 100;   // ignore steps where the wheel didn't spin

    /** One measured step. */
    private static class Point {
        final double power, battery, volts, rpm;
        Point(double power, double battery, double rpm) {
            this.power = power;
            this.battery = battery;
            this.volts = power * battery;
            this.rpm = rpm;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Same motor setup and directions as TeleOp
        Shooter shooter = new Shooter(hardwareMap);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        List<Point> points = new ArrayList<>();

        telemetry.addLine("Press START to run. Flywheel will spin up to full speed.");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        for (double power = START_POWER; power <= END_POWER + 1e-6 && opModeIsActive(); power += POWER_STEP) {
            shooter.setOpenLoopPower(power);

            // Settle
            timer.reset();
            while (opModeIsActive() && timer.seconds() < SETTLE_SEC) {
                shooter.update(12.0);   // voltage is unused in open-loop mode
                showStep(power, "settling", shooter.getRawRpm());
            }

            // Sample
            double rpmSum = 0, voltSum = 0;
            int n = 0;
            timer.reset();
            while (opModeIsActive() && timer.seconds() < SAMPLE_SEC) {
                shooter.update(12.0);
                rpmSum  += shooter.getRawRpm();
                voltSum += voltageSensor.getVoltage();
                n++;
                showStep(power, "sampling", shooter.getRawRpm());
            }
            if (n == 0) break;

            Point p = new Point(power, voltSum / n, rpmSum / n);
            if (p.rpm >= MIN_VALID_RPM) points.add(p);
        }

        shooter.stop();
        shooter.update(12.0);

        // ---------- Least-squares fit: volts = kS + kV * rpm ----------
        double kS = 0, kV = 0, r2 = 0;
        int n = points.size();
        if (n >= 2) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (Point p : points) {
                sx += p.rpm; sy += p.volts; sxx += p.rpm * p.rpm; sxy += p.rpm * p.volts;
            }
            kV = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            kS = (sy - kV * sx) / n;

            double meanY = sy / n, ssTot = 0, ssRes = 0;
            for (Point p : points) {
                double predicted = kS + kV * p.rpm;
                ssRes += (p.volts - predicted) * (p.volts - predicted);
                ssTot += (p.volts - meanY) * (p.volts - meanY);
            }
            r2 = ssTot > 0 ? 1 - ssRes / ssTot : 0;
        }

        // ---------- Show results until STOP ----------
        while (opModeIsActive()) {
            telemetry.addLine("=== RESULTS ===");
            if (n >= 2) {
                telemetry.addData("kS (volts)", "%.4f", kS);
                telemetry.addData("kV (volts per RPM)", "%.6f", kV);
                telemetry.addData("R^2 (closer to 1 = better fit)", "%.4f", r2);
            } else {
                telemetry.addLine("Not enough valid points. Raise START_POWER or check encoders.");
            }
            telemetry.addLine("power | battery V | applied V | RPM");
            for (Point p : points) {
                telemetry.addLine(String.format("%.2f | %.2f | %.2f | %.0f", p.power, p.battery, p.volts, p.rpm));
            }
            telemetry.update();
        }
    }

    private void showStep(double power, String phase, double rpm) {
        telemetry.addData("Step", "power %.2f  (%s)", power, phase);
        telemetry.addData("rpm", rpm);
        telemetry.update();
    }
}
