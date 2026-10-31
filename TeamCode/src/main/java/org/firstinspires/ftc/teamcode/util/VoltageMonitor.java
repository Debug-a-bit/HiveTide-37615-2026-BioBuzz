package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Battery voltage, read only every READ_INTERVAL_SEC (each read costs loop time)
 * and smoothed so short sags under load don't cause over-correction.
 */
@Config
public class VoltageMonitor {

    public static double READ_INTERVAL_SEC = 0.5;
    public static double SMOOTHING = 0.3;   // 0..1, higher = follows new readings faster

    private final VoltageSensor sensor;
    private final ElapsedTime timer = new ElapsedTime();
    private double voltage;

    public VoltageMonitor(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        voltage = sensor.getVoltage();
    }

    /** Call once per loop. Returns the latest smoothed voltage. */
    public double update() {
        if (timer.seconds() >= READ_INTERVAL_SEC) {
            voltage += SMOOTHING * (sensor.getVoltage() - voltage);
            timer.reset();
        }
        return voltage;
    }

    public double get() {
        return voltage;
    }
}
