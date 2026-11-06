package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.ThriftyCamera;

import java.util.ArrayList;
import java.util.List;

/**
 * Calibrates the camera from the goal's AprilTags. Finds:
 *   - ThriftyCamera.FX/FY      a scale correction, so goalRange is right
 *   - AutoAim.AIM_OFFSET_DEG   so auto-aim holds still when the robot already points at the goal
 *
 * Setup: put a tape line on the floor straight out from the middle of the goal. Pick a fixed point
 * on the robot (e.g. the front bumper) and a fixed point at the goal (e.g. the goal's base) and
 * measure between those two every time. They don't need to be the camera or the tags: the lens is
 * fitted from how the reading CHANGES between stations, so a constant offset doesn't matter.
 *
 * Each station (distances in STATIONS_CM):
 *   1. Place the robot on the line at that distance, pointing straight at the goal
 *      (goalYaw near 0 on the screen means the camera faces the goal square-on).
 *   2. Press A: the robot sits still and averages the camera for SAMPLE_SEC.
 *   X = finish early and show results (needs 2+ stations for the lens fit).
 *
 * Results: press Y to apply for this session (lens changes take effect the next time an OpMode
 * starts). Copy them into ThriftyCamera.java / AutoAim.java to keep them.
 */
@Config
@Autonomous(name = "Camera Calibration", group = "Testing")
public class CameraCalibration extends LinearOpMode {

    /** Distances (cm) to measure at, from your robot reference point to your goal reference point. */
    public static String STATIONS_CM = "90, 150, 210";
    public static double SAMPLE_SEC = 2.0;
    public static int    MIN_FRAMES = 10;   // fewer sightings than this = retry the station

    /** Averaged camera readings at one station. */
    private static class Station {
        final double distance, forward, bearing, yaw;
        final int frames;
        Station(double distance, double forward, double bearing, double yaw, int frames) {
            this.distance = distance;
            this.forward = forward;
            this.bearing = bearing;
            this.yaw = yaw;
            this.frames = frames;
        }
    }

    private ThriftyCamera camera;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new ThriftyCamera(hardwareMap);

        while (opModeInInit()) {
            if (gamepad1.x)      ThriftyCamera.TARGET_GOAL = ThriftyCamera.BLUE_GOAL;
            else if (gamepad1.b) ThriftyCamera.TARGET_GOAL = ThriftyCamera.RED_GOAL;
            camera.update();

            telemetry.addData("Camera", camera.isConnected() ? "connected" : "NOT FOUND (check config name)");
            telemetry.addData("Goal", ThriftyCamera.TARGET_GOAL + "   (X = blue, B = red)");
            telemetry.addData("Goal seen", camera.hasTarget());
            telemetry.addLine("The robot does not drive. Press START.");
            telemetry.update();
        }

        List<Double> distances = parseDistances();
        List<Station> stations = new ArrayList<>();

        for (int i = 0; i < distances.size() && opModeIsActive(); ) {
            double distance = distances.get(i);
            Boolean measure = waitForPlacement(i + 1, distances.size(), distance);
            if (measure == null || !measure) break;   // stopped, or X = finish now

            Station station = sample(distance);
            if (station == null) break;
            if (station.frames < MIN_FRAMES) {
                showMessage(String.format("Only saw the goal %d times at %.0f cm. Check the view and press A to retry.",
                        station.frames, distance));
                continue;   // same station again
            }
            stations.add(station);
            i++;
        }

        showResults(stations);
        camera.close();
    }

    /** Live view while the robot is placed. Returns true = A (measure), false = X (finish), null = stopped. */
    private Boolean waitForPlacement(int number, int total, double distance) {
        while (opModeIsActive() && (gamepad1.a || gamepad1.x)) camera.update();   // wait for release
        while (opModeIsActive()) {
            camera.update();
            if (gamepad1.a) return true;
            if (gamepad1.x) return false;

            telemetry.addData("Station", "%d of %d:  %.0f cm", number, total, distance);
            telemetry.addLine("Place robot on the line, pointing straight at the goal. A = measure, X = finish.");
            showLiveCamera();
            telemetry.update();
        }
        return null;
    }

    /** Averages the camera over SAMPLE_SEC. Returns null if the OpMode was stopped. */
    private Station sample(double distance) {
        double forwardSum = 0, bearingSum = 0, yawSum = 0;
        int frames = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SAMPLE_SEC) {
            camera.update();
            if (camera.isNewTarget()) {   // count each camera frame once
                forwardSum += camera.getTargetForward();
                bearingSum += camera.getTargetBearing();
                yawSum += camera.getTargetYaw();
                frames++;
            }
            telemetry.addData("Measuring", "%.0f cm   (%d frames)  keep still", distance, frames);
            telemetry.update();
        }
        if (!opModeIsActive()) return null;
        if (frames == 0) return new Station(distance, 0, 0, 0, 0);
        return new Station(distance, forwardSum / frames, bearingSum / frames, yawSum / frames, frames);
    }

    private void showResults(List<Station> stations) {
        int n = stations.size();

        // ---- Lens: fit  measuredForward = scale * trueDistance + offset ----
        // A wrong focal length scales every distance by the same factor, so scale = 1 means the lens is right.
        double scale = 1, intercept = 0, r2 = 0;
        boolean lensFitted = false;
        if (n >= 2) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (Station s : stations) {
                sx += s.distance; sy += s.forward; sxx += s.distance * s.distance; sxy += s.distance * s.forward;
            }
            double denominator = n * sxx - sx * sx;
            if (denominator > 1e-6) {
                scale = (n * sxy - sx * sy) / denominator;
                intercept = (sy - scale * sx) / n;
                double meanY = sy / n, ssTot = 0, ssRes = 0;
                for (Station s : stations) {
                    double predicted = scale * s.distance + intercept;
                    ssRes += (s.forward - predicted) * (s.forward - predicted);
                    ssTot += (s.forward - meanY) * (s.forward - meanY);
                }
                r2 = ssTot > 0 ? 1 - ssRes / ssTot : 0;
                lensFitted = scale > 0;
            }
        }

        // ---- Aim offset: bearing when pointing straight at the goal, corrected for the new lens ----
        // (sideways positions don't change with focal length but forward ones do, so tan(bearing) scales)
        double bearingSum = 0, bearingMin = Double.MAX_VALUE, bearingMax = -Double.MAX_VALUE;
        double[] corrected = new double[n];
        for (int i = 0; i < n; i++) {
            corrected[i] = Math.toDegrees(Math.atan(scale * Math.tan(Math.toRadians(stations.get(i).bearing))));
            bearingSum += corrected[i];
            bearingMin = Math.min(bearingMin, corrected[i]);
            bearingMax = Math.max(bearingMax, corrected[i]);
        }
        double aimOffset = n > 0 ? -bearingSum / n : 0;

        boolean applied = false;
        while (opModeIsActive()) {
            if (gamepad1.y && !applied && n > 0) {
                if (lensFitted) {
                    ThriftyCamera.FX /= scale;
                    ThriftyCamera.FY /= scale;
                }
                AutoAim.AIM_OFFSET_DEG = Math.round(aimOffset * 10) / 10.0;
                applied = true;
            }

            telemetry.addLine("=== RESULTS ===");
            if (n == 0) {
                telemetry.addLine("No stations measured.");
            } else {
                if (lensFitted) {
                    telemetry.addData("Distance scale", "%.3f  (1.000 = lens already right)", scale);
                    telemetry.addData("Fit R^2", "%.4f  (closer to 1 = better)", r2);
                    telemetry.addData("FX / FY", "%.1f / %.1f   (now %.1f / %.1f)",
                            ThriftyCamera.FX / scale, ThriftyCamera.FY / scale, ThriftyCamera.FX, ThriftyCamera.FY);
                } else {
                    telemetry.addLine("Lens not fitted: needs 2+ stations at different distances.");
                }
                telemetry.addData("AIM_OFFSET_DEG", "%.1f   (now %.1f)", aimOffset, AutoAim.AIM_OFFSET_DEG);
                if (n >= 2 && bearingMax - bearingMin > 2) {
                    telemetry.addLine(String.format("NOTE: bearing changed %.1f deg between distances, so the camera sits "
                            + "off-centre sideways. The offset is averaged; aim is best near the middle distance.",
                            bearingMax - bearingMin));
                }
                telemetry.addLine(applied ? "Applied for this session (lens: next OpMode start). Copy into the code to keep."
                                          : "Y = apply for this session.");
            }

            telemetry.addLine("true cm | measured fwd | bearing (corrected) | yaw | frames");
            for (int i = 0; i < n; i++) {
                Station s = stations.get(i);
                telemetry.addLine(String.format("%.0f | %.1f | %.1f (%.1f) | %.1f | %d",
                        s.distance, s.forward, s.bearing, corrected[i], s.yaw, s.frames));
            }
            telemetry.update();
        }
    }

    private void showLiveCamera() {
        telemetry.addData("Camera", camera.isConnected() ? "connected" : "NOT FOUND");
        telemetry.addData("Goal seen", camera.hasTarget());
        telemetry.addData("goalForward (cm)", "%.1f", camera.getTargetForward());
        telemetry.addData("goalBearing (deg)", "%.1f", camera.getTargetBearing());
        telemetry.addData("goalYaw (deg)", "%.1f   (near 0 = square to the goal)", camera.getTargetYaw());
    }

    private void showMessage(String message) {
        telemetry.addLine(message);
        telemetry.update();
        sleep(1500);
    }

    private List<Double> parseDistances() {
        List<Double> distances = new ArrayList<>();
        for (String part : STATIONS_CM.split(",")) {
            try {
                distances.add(Double.parseDouble(part.trim()));
            } catch (NumberFormatException ignored) {
                // skip anything that isn't a number
            }
        }
        return distances;
    }
}
