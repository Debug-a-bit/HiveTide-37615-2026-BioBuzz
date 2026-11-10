package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auto.AutoActions.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.auto.AutoActions.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.auto.AutoActions.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.auto.AutoActions.Direction.RIGHT;
import static org.firstinspires.ftc.teamcode.auto.AutoActions.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.auto.AutoActions.INTAKE_ON;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoActions;
import org.firstinspires.ftc.teamcode.subsystems.ThriftyCamera;

/**
 * During init: X = blue alliance, B = red alliance (picks which goal to aim at).
 *
 * Starts preloaded in front of the goal:
 *   1. Aim at the goal and shoot the preloaded balls.
 *   2. Turn PICKUP_TURN_DEG and drive backward with the intake on, wiggling to pull the last balls in.
 *   3. Straighten up, drive back to the shooting spot, turning another PICKUP_TURN_DEG on the way.
 *   4. Aim and shoot again, then move off.
 *
 * Moves are timed and field-centric: "forward" etc. stay the same field direction whichever way the
 * robot is facing. Directions are relative to how the robot sits at START.
 * The actions themselves live in AutoActions, so other autos can reuse them.
 *
 * Naming: *_DIST values are distances, measured in seconds of driving at MOVEMENT_SPEED (a slower move
 * covers the same distance, it just takes longer). *_TIME and *_TIMEOUT values are real seconds that an
 * action (pausing, aiming, shooting, turning) takes or is allowed to take.
 */
@Config
@Autonomous(name = "Auto", group = "Testing")
public class Auto extends LinearOpMode {

    // ---- Distances (seconds of driving at MOVEMENT_SPEED) ----
    /** Left, away from the shooting spot before the first pickup turn. */
    public static double LEAVE_SHOOT_LEFT_DIST = 0.2;
    /** Right, lining up with the balls after the first pickup turn. */
    public static double PICKUP_LINEUP_RIGHT_DIST = 0.5;
    /** Backward into the balls with the intake on; the return forward undoes it. */
    public static double PICKUP_DEPTH_DIST = 1.25;
    /** How much of PICKUP_DEPTH_DIST is driven at MOVEMENT_SPEED; the rest is at INTAKE_MOVEMENT_SPEED. */
    public static double PICKUP_FAST_PART_DIST = 0.52;
    /** Short move forward out of the pickup before straightening up. */
    public static double PICKUP_EXIT_FORWARD_DIST = 0.4;
    /** Left, from the pickup back across to the shooting spot. */
    public static double RETURN_LEFT_DIST = 2.8;
    /** How far the robot drives after the pickup pause with the intake still running (the exit forward, then the start of the return). */
    public static double INTAKE_AFTER_PAUSE_DIST = 2.0;
    /** Small move left at the shooting spot, just before aiming for the final volley. */
    public static double FINAL_LEFT_DIST = 0.15;
    /** After the final volley: right, then backward. */
    public static double END_RIGHT_DIST = 0.15;
    public static double END_BACK_DIST = 1.25;
    /** Each wiggle push, at WIGGLE_SPEED. */
    public static double WIGGLE_EACH_WAY_DIST = 0.1;
    /** After the wiggle: right, then backward into the wall to square up, at WALL_ALIGN_SPEED. */
    public static double WALL_ALIGN_RIGHT_DIST = 0.15;
    public static double WALL_ALIGN_BACK_DIST = 0.15;

    // ---- Intake actions ----
    /** Seconds stopped with the intake on after the wiggle. */
    public static double PICKUP_PAUSE_TIME = 1.5;
    public static int WIGGLE_COUNT = 3;
    /** Seconds stopped (intake still on) at each change of direction: slows the wiggle without weakening each push. */
    public static double WIGGLE_PAUSE_TIME = 0.25;

    // ---- Aiming, shooting and turning actions (seconds) ----
    /** Time spent turning toward the goal before the first volley (starts roughly lined up). */
    public static double FIRST_AIM_TIME = 1.0;
    /** Time spent turning toward the goal before the second volley. */
    public static double SECOND_AIM_TIME = 3.0;
    /** Longest a volley may take (includes spinning up the flywheel). */
    public static double SHOOT_TIMEOUT = 3.0;
    /** Longest a turn may take if it can't get within FieldCentricDrive.ALIGN_TOLERANCE_DEG. */
    public static double TURN_TIMEOUT = 1.5;
    /** Size of each pickup turn (degrees, + = counter-clockwise). */
    public static double PICKUP_TURN_DEG = -90;

    // ---- Speeds (0 to 1) ----
    public static double MOVEMENT_SPEED = 0.6;
    public static double INTAKE_MOVEMENT_SPEED = 0.25;
    /** Power of each wiggle push. */
    public static double WIGGLE_SPEED = 0.25;
    /** Gentle, so squaring up against the wall doesn't bang into it. */
    public static double WALL_ALIGN_SPEED = 0.4;
    public static double SLOW_RIGHT_SPEED = 0.3;

    /** How often (ms) the Dashboard graph gets a new point. Lower = smoother graph, more Wi-Fi traffic. */
    public static int DASHBOARD_INTERVAL_MS = 20;

    private AutoActions robot;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(DASHBOARD_INTERVAL_MS);   // default is 100 ms
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new AutoActions(this);

        // Pick alliance (which goal to aim at) until START is pressed
        while (opModeInInit()) {
            if (gamepad1.x)      ThriftyCamera.ALLIANCE = ThriftyCamera.BLUE;
            else if (gamepad1.b) ThriftyCamera.ALLIANCE = ThriftyCamera.RED;
            robot.camera.update();   // auto brightness tunes while waiting

            telemetry.addData("Status", "Initialized. Ready.");
            telemetry.addData("Camera", robot.camera.isConnected() ? "connected" : "NOT FOUND (check config name)");
            telemetry.addData("Brightness", robot.camera.getBrightnessStatus());
            telemetry.addData("Aiming at", ThriftyCamera.ALLIANCE + "   (X = blue, B = red)");
            telemetry.update();
        }
        if (isStopRequested()) return;
        robot.camera.lockBrightness();
        robot.markStart();
        robot.setNormalSpeed(MOVEMENT_SPEED);
        robot.alignToGoal(FIRST_AIM_TIME);
        robot.shoot(SHOOT_TIMEOUT);
        robot.move(LEFT, 0.5, INTAKE_OFF);
        robot.move(BACKWARD, 0.85, INTAKE_OFF);
        robot.move(LEFT, 2.05, INTAKE_OFF);

        robot.stopAll();
    }
}