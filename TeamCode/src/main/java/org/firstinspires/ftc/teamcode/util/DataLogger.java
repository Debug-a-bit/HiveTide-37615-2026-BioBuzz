package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes one CSV row per loop to the Control Hub, so every loop is kept
 * (the Dashboard graph only shows the samples it manages to send).
 *
 * Files land in /sdcard/FIRST/logs/<name>_<date>_<time>.csv
 * Copy them to the laptop with:   adb pull /sdcard/FIRST/logs
 * then open in Excel / Google Sheets and graph the columns.
 *
 * Writing is buffered, so it costs very little loop time. Call close() at the end
 * of the OpMode or the last rows may be lost.
 */
public class DataLogger {

    private BufferedWriter writer;
    private final long startNanos = System.nanoTime();
    private final StringBuilder row = new StringBuilder();

    /** @param columns column names, in the same order you'll pass values to log() */
    public DataLogger(String name, String... columns) {
        try {
            File dir = new File(AppUtil.ROOT_FOLDER, "logs");
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            writer = new BufferedWriter(new FileWriter(new File(dir, name + "_" + stamp + ".csv")));

            writer.write("timeSec");
            for (String column : columns) writer.write("," + column);
            writer.newLine();
        } catch (IOException e) {
            RobotLog.ee("DataLogger", e, "Could not open log file; logging disabled");
            writer = null;
        }
    }

    /** Adds one row. Time since the logger was created is added as the first column. */
    public void log(double... values) {
        if (writer == null) return;
        row.setLength(0);
        row.append(String.format(Locale.US, "%.4f", (System.nanoTime() - startNanos) / 1e9));
        for (double value : values) row.append(',').append(value);
        try {
            writer.write(row.toString());
            writer.newLine();
        } catch (IOException e) {
            RobotLog.ee("DataLogger", e, "Write failed; logging disabled");
            close();
        }
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException ignored) {
        }
        writer = null;
    }
}
