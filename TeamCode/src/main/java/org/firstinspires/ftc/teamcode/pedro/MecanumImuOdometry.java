package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Odometry from the 4 mecanum drive encoders (x/y) + the Control Hub IMU (heading).
 * Pure FTC SDK - no Pedro classes - so it compiles on any Pedro version.
 *
 * Field frame: +x forward, +y left, heading CCW-positive, inches / radians.
 */
public class MecanumImuOdometry {

    public static class Config {
        // Must match your robot configuration / MecanumConfig names
        public String frontLeftName = "frontLeftMotor";
        public String frontRightName = "frontRightMotor";
        public String backLeftName = "backLeftMotor";
        public String backRightName = "backRightMotor";
        public String imuName = "imu";

        // Physical orientation of the Control Hub
        public RevHubOrientationOnRobot.LogoFacingDirection logo =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usb =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // Set both to 1.0 for calibration, then replace with tuned values
        public double forwardInchesPerTick = 1.0;
        public double strafeInchesPerTick = 1.0;

        // +1 or -1. With motor directions set so forward power drives forward,
        // these are usually all +1. Flip any encoder that counts DOWN when pushed forward.
        public int frontLeftSign = 1;
        public int frontRightSign = 1;
        public int backLeftSign = 1;
        public int backRightSign = 1;
    }

    private final Config c;
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

    private boolean firstUpdate = true;
    private int lastFl, lastFr, lastBl, lastBr;
    private double lastYaw;
    private long lastNanos;

    private double x, y, heading;       // field pose
    private double headingOffset;       // field heading = IMU yaw + offset
    private double vForward, vStrafe, omega;  // robot-frame velocity (in/s, rad/s)

    public MecanumImuOdometry(HardwareMap hardwareMap, Config config) {
        this.c = config;
        fl = hardwareMap.get(DcMotorEx.class, c.frontLeftName);
        fr = hardwareMap.get(DcMotorEx.class, c.frontRightName);
        bl = hardwareMap.get(DcMotorEx.class, c.backLeftName);
        br = hardwareMap.get(DcMotorEx.class, c.backRightName);

        imu = hardwareMap.get(IMU.class, c.imuName);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(c.logo, c.usb)));
        imu.resetYaw();
    }

    private double readYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Call exactly once per loop. Reads each encoder and the IMU once. */
    public void update() {
        int pFl = fl.getCurrentPosition() * c.frontLeftSign;
        int pFr = fr.getCurrentPosition() * c.frontRightSign;
        int pBl = bl.getCurrentPosition() * c.backLeftSign;
        int pBr = br.getCurrentPosition() * c.backRightSign;
        double yaw = readYaw();
        long now = System.nanoTime();

        // Baseline on the first update (not in the constructor), so motor
        // directions applied later by the drivetrain don't create a fake jump.
        if (firstUpdate) {
            lastFl = pFl; lastFr = pFr; lastBl = pBl; lastBr = pBr;
            lastYaw = yaw;
            lastNanos = now;
            heading = AngleUnit.normalizeRadians(yaw + headingOffset);
            firstUpdate = false;
            return;
        }

        int dFl = pFl - lastFl, dFr = pFr - lastFr, dBl = pBl - lastBl, dBr = pBr - lastBr;
        lastFl = pFl; lastFr = pFr; lastBl = pBl; lastBr = pBr;

        // Mecanum forward kinematics (robot frame, +strafe = left)
        double dF = (dFl + dFr + dBl + dBr) / 4.0 * c.forwardInchesPerTick;
        double dS = (-dFl + dFr + dBl - dBr) / 4.0 * c.strafeInchesPerTick;

        // Heading change from the IMU, not the wheels
        double dTheta = AngleUnit.normalizeRadians(yaw - lastYaw);
        lastYaw = yaw;

        // Pose exponential: integrate along an arc instead of a straight line
        double sinTerm, cosTerm;
        if (Math.abs(dTheta) < 1e-6) {
            sinTerm = 1.0 - dTheta * dTheta / 6.0;
            cosTerm = dTheta / 2.0;
        } else {
            sinTerm = Math.sin(dTheta) / dTheta;
            cosTerm = (1.0 - Math.cos(dTheta)) / dTheta;
        }
        double dxRobot = dF * sinTerm - dS * cosTerm;
        double dyRobot = dF * cosTerm + dS * sinTerm;

        double cosH = Math.cos(heading), sinH = Math.sin(heading);
        x += dxRobot * cosH - dyRobot * sinH;
        y += dxRobot * sinH + dyRobot * cosH;
        heading = AngleUnit.normalizeRadians(yaw + headingOffset);

        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt > 1e-4) {
            vForward = dF / dt;
            vStrafe = dS / dt;
            omega = dTheta / dt;
        }
    }

    /** Teleport the pose estimate (e.g. start pose). Heading stays locked to the IMU. */
    public void setPose(double newX, double newY, double newHeading) {
        x = newX;
        y = newY;
        heading = AngleUnit.normalizeRadians(newHeading);
        headingOffset = newHeading - readYaw();
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    /** Robot-frame velocity (what Pedro calls a twist). */
    public double getForwardVelocity() { return vForward; }
    public double getStrafeVelocity() { return vStrafe; }
    public double getAngularVelocity() { return omega; }

    /** Field-frame velocity. */
    public double getFieldVx() { return vForward * Math.cos(heading) - vStrafe * Math.sin(heading); }
    public double getFieldVy() { return vForward * Math.sin(heading) + vStrafe * Math.cos(heading); }
}
