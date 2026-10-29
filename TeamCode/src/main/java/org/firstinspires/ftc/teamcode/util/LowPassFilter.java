package org.firstinspires.ftc.teamcode.util;

/**
 * Time-based exponential low-pass filter.
 * The delay it adds stays roughly equal to timeConstantSec,
 * no matter how fast or slow the loop runs.
 * Call update() once per loop with the newest value.
 */
public class LowPassFilter {
    private double timeConstantSec;
    private double value = 0.0;
    private boolean initialized = false;
    private long lastNanos = 0;

    public LowPassFilter(double timeConstantSec) {
        this.timeConstantSec = timeConstantSec;
    }

    /** Adds a new sample and returns the filtered value. */
    public double update(double input) {
        long now = System.nanoTime();
        if (!initialized) {              // start at the first reading, not at 0
            value = input;
            lastNanos = now;
            initialized = true;
            return value;
        }
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        double alpha = (timeConstantSec <= 0) ? 1.0 : 1.0 - Math.exp(-dt / timeConstantSec);
        value += alpha * (input - value);
        return value;
    }

    /** Returns the current filtered value without adding a sample. */
    public double get() {
        return value;
    }

    /** Lets you tune the time constant live (e.g. from FTC Dashboard). */
    public void setTimeConstant(double timeConstantSec) {
        this.timeConstantSec = timeConstantSec;
    }

    /** Clears the filter; the next update() starts fresh from that reading. */
    public void reset() {
        initialized = false;
    }
}
