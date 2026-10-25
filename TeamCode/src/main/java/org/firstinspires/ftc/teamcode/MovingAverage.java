package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

/**
 * Simple moving average over the last N samples.
 * Call update() once per loop with the newest value; it returns the current average.
 */
public class MovingAverage {
    private final double[] samples;
    private int index = 0;      // where the next sample is written (overwrites the oldest)
    private int count = 0;      // how many samples are stored so far (up to the window size)
    private double sum = 0.0;   // running total of the stored samples

    public MovingAverage(int windowSize) {
        if (windowSize < 1) {
            throw new IllegalArgumentException("windowSize must be at least 1");
        }
        samples = new double[windowSize];
    }

    /** Adds a new sample and returns the average of the stored samples. */
    public double update(double value) {
        sum -= samples[index];                      // remove the oldest sample (0 until the window fills)
        samples[index] = value;
        sum += value;
        index = (index + 1) % samples.length;
        if (count < samples.length) {
            count++;
        }
        return sum / count;                         // until the window fills, averages what it has
    }

    /** Returns the current average without adding a sample. */
    public double get() {
        return count == 0 ? 0.0 : sum / count;
    }

    /** Clears all stored samples. */
    public void reset() {
        Arrays.fill(samples, 0.0);
        index = 0;
        count = 0;
        sum = 0.0;
    }
}