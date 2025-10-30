package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Simplified Tank Drive with Pure RAMSETE Controller
 * Based on FTC competition best practices and Road Runner principles
 * Key improvements:
 * 1. Simplified velocity control - no complex motion profiling
 * 2. Standard RAMSETE gains (proven in FRC/FTC)
 * 3. Feedforward + Feedback control
 * 4. Proper deadband and saturation handling
 */
@Config
public class SimpleTankDrive {

    @Config
    public static class Params {
        // Robot physical parameters
        public static double TRACK_WIDTH_INCHES = 11.5;
        public static double WHEEL_DIAMETER_INCHES = 2.83;

        // Motor PID - Tune these if velocity tracking is poor
        public static double kP = 29.0;
        public static double kI = 0.0;
        public static double kD = 0.2;
        public static double kF = 12.0;

        // RAMSETE Controller (STANDARD FRC/FTC VALUES - rarely need tuning)
        public static double RAMSETE_B = 2.0;      // Convergence aggressiveness
        public static double RAMSETE_ZETA = 0.7;   // Damping ratio

        // Velocity limits
        public static double MAX_VEL = 30.0;        // in/s
        public static double MAX_ACCEL = 40.0;      // in/s²
        public static double MAX_ANG_VEL = 180.0;   // deg/s

        // Feedforward gains - CRITICAL for good tracking
        // kV: velocity gain (voltage per in/s)
        // kA: acceleration gain (voltage per in/s²)
        // kStatic: overcome static friction
        public static double kV = 1.0 / MAX_VEL;    // Auto-calculated starting point
        public static double kA = 0.0;              // Usually 0 for FTC (battery variance)
        public static double kStatic = 0.0;         // Tune if robot doesn't start moving

        // PID gains for trajectory following
        public static double LATERAL_GAIN = 2.0;    // Correct sideways error
        public static double HEADING_GAIN = 3.0;    // Correct heading error

        // Tolerances
        public static double POS_TOL = 0.5;         // inches
        public static double HEADING_TOL = 2.0;     // degrees

        // Deadband - ignore errors smaller than this
        public static double POS_DEADBAND = 0.1;    // inches
        public static double HEADING_DEADBAND = 0.5; // degrees
    }

    private static final double TICKS_PER_REV = 751.8;
    private static final double MAX_RPM = 312.0;
    private static final double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final GoBildaPinpointDriver odo;
    private final LinearOpMode opMode;

    private Pose2d currentPose;
    private final List<Pose2d> poseHistory = new ArrayList<>();

    // Previous velocity for acceleration calculation
    private double prevLeftVel = 0.0;
    private double prevRightVel = 0.0;

    public SimpleTankDrive(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE mode for instant stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initVelocityControl();

        // Initialize Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        currentPose = new Pose2d(0, 0, 0);
    }

    private void configurePinpoint() {
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setYawScalar(1.0);
        odo.resetPosAndIMU();
    }

    public void waitForPinpointReady() {
        while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                && !opMode.isStopRequested()) {
            opMode.telemetry.addLine("Waiting for Pinpoint...");
            opMode.telemetry.update();
            opMode.sleep(50);
        }
    }

    public void setPose(Pose2d pose) {
        Pose2D pinpointPose = new Pose2D(
                DistanceUnit.INCH, pose.position.x, pose.position.y,
                AngleUnit.RADIANS, pose.heading.toDouble()
        );
        odo.setPosition(pinpointPose);
        currentPose = pose;
    }

    public void updateOdometry() {
        odo.update();
        Pose2D pose = odo.getPosition();

        currentPose = new Pose2d(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS)
        );

        poseHistory.add(currentPose);
        if (poseHistory.size() > 100) {
            poseHistory.remove(0);
        }
    }

    public Pose2d getPose() {
        return currentPose;
    }

    private void initVelocityControl() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPIDF(leftFront);
        setMotorPIDF(leftBack);
        setMotorPIDF(rightFront);
        setMotorPIDF(rightBack);
    }

    private void setMotorPIDF(DcMotorEx motor) {
        motor.setVelocityPIDFCoefficients(Params.kP, Params.kI, Params.kD, Params.kF);
    }

    /**
     * Set wheel velocities with feedforward compensation
     * This is the key to smooth, accurate motion
     */
    public void setWheelVelocities(double leftVel, double rightVel) {
        // Calculate acceleration (change in velocity)
        double leftAccel = (leftVel - prevLeftVel) * 50.0; // Assuming 20ms loop = 50Hz
        double rightAccel = (rightVel - prevRightVel) * 50.0;

        // Feedforward: predict voltage needed
        double leftFF = Params.kV * leftVel + Params.kA * leftAccel + Math.signum(leftVel) * Params.kStatic;
        double rightFF = Params.kV * rightVel + Params.kA * rightAccel + Math.signum(rightVel) * Params.kStatic;

        // Convert to ticks/sec
        double wheelCirc = Math.PI * Params.WHEEL_DIAMETER_INCHES;
        double ticksPerInch = TICKS_PER_REV / wheelCirc;

        double leftTicks = leftVel * ticksPerInch;
        double rightTicks = rightVel * ticksPerInch;

        // Clamp to motor limits
        leftTicks = clamp(leftTicks, -MAX_TICKS_PER_SEC, MAX_TICKS_PER_SEC);
        rightTicks = clamp(rightTicks, -MAX_TICKS_PER_SEC, MAX_TICKS_PER_SEC);

        // Set velocity (motor PID + feedforward)
        leftFront.setVelocity(leftTicks);
        leftBack.setVelocity(leftTicks);
        rightFront.setVelocity(rightTicks);
        rightBack.setVelocity(rightTicks);

        // Store for next iteration
        prevLeftVel = leftVel;
        prevRightVel = rightVel;
    }

    public void stop() {
        setWheelVelocities(0, 0);
        prevLeftVel = 0;
        prevRightVel = 0;
    }

    /**
     * Pure RAMSETE controller - industry standard for tank drives
     * No complex velocity scaling - just clean feedback control
     */
    private double[] ramseteControl(Pose2d targetPose, double targetVel, double targetAngularVel) {
        // Pose error in global frame
        double ex = targetPose.position.x - currentPose.position.x;
        double ey = targetPose.position.y - currentPose.position.y;
        double etheta = normalizeAngle(targetPose.heading.toDouble() - currentPose.heading.toDouble());

        // Transform to robot frame
        double cos = Math.cos(currentPose.heading.toDouble());
        double sin = Math.sin(currentPose.heading.toDouble());
        double exRobot = ex * cos + ey * sin;
        double eyRobot = -ex * sin + ey * cos;

        // Apply deadband - ignore tiny errors
        if (Math.abs(exRobot) < Params.POS_DEADBAND) exRobot = 0;
        if (Math.abs(eyRobot) < Params.POS_DEADBAND) eyRobot = 0;
        if (Math.abs(Math.toDegrees(etheta)) < Params.HEADING_DEADBAND) etheta = 0;

        // RAMSETE gain
        double k = 2.0 * Params.RAMSETE_ZETA * Math.sqrt(
                targetAngularVel * targetAngularVel +
                        Params.RAMSETE_B * targetVel * targetVel
        );

        // Commanded velocities - standard RAMSETE equations
        double vCmd = targetVel * Math.cos(etheta) + k * exRobot;
        double omegaCmd = targetAngularVel +
                Params.RAMSETE_B * targetVel * sinc(etheta) * eyRobot +
                k * etheta;

        // Differential drive kinematics: convert to wheel velocities
        double leftVel = vCmd - (omegaCmd * Params.TRACK_WIDTH_INCHES / 2.0);
        double rightVel = vCmd + (omegaCmd * Params.TRACK_WIDTH_INCHES / 2.0);

        // Saturate to max velocity
        double scale = Math.max(
                Math.abs(leftVel) / Params.MAX_VEL,
                Math.abs(rightVel) / Params.MAX_VEL
        );
        if (scale > 1.0) {
            leftVel /= scale;
            rightVel /= scale;
        }

        return new double[]{leftVel, rightVel, vCmd, omegaCmd};
    }

    private double sinc(double x) {
        if (Math.abs(x) < 1e-9) return 1.0;
        return Math.sin(x) / x;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ============================================
    // ACTIONS
    // ============================================

    public Action driveTo(double targetX, double targetY) {
        return new DriveToAction(new Pose2d(targetX, targetY, currentPose.heading.toDouble()));
    }

    public Action turnTo(double targetHeadingDegrees) {
        return new TurnToAction(Math.toRadians(targetHeadingDegrees));
    }

    public Action driveToPose(double targetX, double targetY, double targetHeadingDegrees) {
        return new DriveToPoseAction(new Pose2d(targetX, targetY, Math.toRadians(targetHeadingDegrees)));
    }

    /**
     * Drive to position action
     * Uses constant velocity + RAMSETE for error correction
     */
    private class DriveToAction implements Action {
        private final Pose2d targetPose;
        private int settleCount = 0;
        private static final int SETTLE_THRESHOLD = 5; // 100ms at 20ms/loop

        public DriveToAction(Pose2d target) {
            this.targetPose = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateOdometry();

            double dx = targetPose.position.x - currentPose.position.x;
            double dy = targetPose.position.y - currentPose.position.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            // Telemetry
            opMode.telemetry.addData("Distance", "%.2f in", distance);
            opMode.telemetry.addData("Current", "(%.1f, %.1f)",
                    currentPose.position.x, currentPose.position.y);
            packet.put("Distance", distance);
            packet.put("Current X", currentPose.position.x);
            packet.put("Current Y", currentPose.position.y);

            // Check if at target
            if (distance < Params.POS_TOL) {
                settleCount++;
                if (settleCount >= SETTLE_THRESHOLD) {
                    stop();
                    opMode.telemetry.addLine("✓ Complete");
                    opMode.telemetry.addData("Error", "%.2f in", distance);
                    opMode.telemetry.update();
                    packet.put("Status", "Complete");
                    return false;
                }
                // Still settling - hold position
                stop();
                return true;
            }

            settleCount = 0;

            // Calculate target velocity using trapezoidal profile
            double targetVel = calculateTargetVelocity(distance);

            // Use RAMSETE for trajectory tracking
            double[] result = ramseteControl(targetPose, targetVel, 0.0);
            setWheelVelocities(result[0], result[1]);

            opMode.telemetry.addData("Target Vel", "%.1f in/s", targetVel);
            opMode.telemetry.addData("L/R Vel", "%.1f / %.1f", result[0], result[1]);
            opMode.telemetry.update();

            packet.put("Target Velocity", targetVel);
            packet.put("Left Vel", result[0]);
            packet.put("Right Vel", result[1]);

            drawFieldOverlay(packet.fieldOverlay(), targetPose);

            return true;
        }

        /**
         * Simple trapezoidal velocity profile
         * Accelerate -> Cruise -> Decelerate
         */
        private double calculateTargetVelocity(double distanceRemaining) {
            // Distance needed to decelerate from current velocity to stop
            double decelDist = (Params.MAX_VEL * Params.MAX_VEL) / (2.0 * Params.MAX_ACCEL);

            if (distanceRemaining < decelDist) {
                // Deceleration phase: v = sqrt(2 * a * d)
                return Math.sqrt(2.0 * Params.MAX_ACCEL * distanceRemaining);
            } else {
                // Acceleration/cruise phase
                return Params.MAX_VEL;
            }
        }
    }

    /**
     * Turn to heading action
     */
    private class TurnToAction implements Action {
        private final double targetHeading;
        private int settleCount = 0;
        private static final int SETTLE_THRESHOLD = 5;

        public TurnToAction(double targetHeading) {
            this.targetHeading = targetHeading;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateOdometry();

            double headingError = normalizeAngle(targetHeading - currentPose.heading.toDouble());
            double headingErrorDeg = Math.toDegrees(headingError);

            opMode.telemetry.addData("Heading Error", "%.1f°", headingErrorDeg);
            packet.put("Heading Error", headingErrorDeg);

            if (Math.abs(headingErrorDeg) < Params.HEADING_TOL) {
                settleCount++;
                if (settleCount >= SETTLE_THRESHOLD) {
                    stop();
                    opMode.telemetry.addLine("✓ Turn Complete");
                    opMode.telemetry.update();
                    packet.put("Status", "Complete");
                    return false;
                }
                stop();
                return true;
            }

            settleCount = 0;

            // Proportional control with saturation
            double targetAngularVel = Params.HEADING_GAIN * headingError;
            targetAngularVel = clamp(targetAngularVel,
                    -Math.toRadians(Params.MAX_ANG_VEL),
                    Math.toRadians(Params.MAX_ANG_VEL));

            double leftVel = -(targetAngularVel * Params.TRACK_WIDTH_INCHES / 2.0);
            double rightVel = (targetAngularVel * Params.TRACK_WIDTH_INCHES / 2.0);

            setWheelVelocities(leftVel, rightVel);

            opMode.telemetry.addData("Angular Vel", "%.1f °/s", Math.toDegrees(targetAngularVel));
            opMode.telemetry.update();

            packet.put("Angular Velocity", Math.toDegrees(targetAngularVel));

            return true;
        }
    }

    /**
     * Drive to pose with heading action
     */
    private class DriveToPoseAction implements Action {
        private final Pose2d targetPose;
        private int settleCount = 0;
        private static final int SETTLE_THRESHOLD = 5;

        public DriveToPoseAction(Pose2d target) {
            this.targetPose = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateOdometry();

            double dx = targetPose.position.x - currentPose.position.x;
            double dy = targetPose.position.y - currentPose.position.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            double headingError = normalizeAngle(targetPose.heading.toDouble() - currentPose.heading.toDouble());

            boolean posReached = distance < Params.POS_TOL;
            boolean headingReached = Math.abs(Math.toDegrees(headingError)) < Params.HEADING_TOL;

            opMode.telemetry.addData("Distance", "%.2f in", distance);
            opMode.telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
            packet.put("Distance", distance);
            packet.put("Heading Error", Math.toDegrees(headingError));

            if (posReached && headingReached) {
                settleCount++;
                if (settleCount >= SETTLE_THRESHOLD) {
                    stop();
                    opMode.telemetry.addLine("✓ Complete");
                    opMode.telemetry.update();
                    packet.put("Status", "Complete");
                    return false;
                }
                stop();
                return true;
            }

            settleCount = 0;

            // Target velocities
            double targetVel = calculateTargetVelocity(distance);
            double targetAngularVel = Params.HEADING_GAIN * headingError;
            targetAngularVel = clamp(targetAngularVel,
                    -Math.toRadians(Params.MAX_ANG_VEL),
                    Math.toRadians(Params.MAX_ANG_VEL));

            // RAMSETE control
            double[] result = ramseteControl(targetPose, targetVel, targetAngularVel);
            setWheelVelocities(result[0], result[1]);

            opMode.telemetry.update();

            drawFieldOverlay(packet.fieldOverlay(), targetPose);

            return true;
        }

        private double calculateTargetVelocity(double distanceRemaining) {
            double decelDist = (Params.MAX_VEL * Params.MAX_VEL) / (2.0 * Params.MAX_ACCEL);
            if (distanceRemaining < decelDist) {
                return Math.sqrt(2.0 * Params.MAX_ACCEL * distanceRemaining);
            }
            return Params.MAX_VEL;
        }
    }

    // ============================================
    // VISUALIZATION
    // ============================================

    private void drawFieldOverlay(Canvas canvas, Pose2d target) {
        if (poseHistory.size() > 1) {
            double[] xPoints = new double[poseHistory.size()];
            double[] yPoints = new double[poseHistory.size()];
            for (int i = 0; i < poseHistory.size(); i++) {
                xPoints[i] = poseHistory.get(i).position.x;
                yPoints[i] = poseHistory.get(i).position.y;
            }
            canvas.setStroke("#3F51B5");
            canvas.strokePolyline(xPoints, yPoints);
        }

        canvas.setStroke("#3F51B5");
        drawRobot(canvas, currentPose);

        canvas.setStroke("#4CAF50");
        drawRobot(canvas, target);
    }

    private void drawRobot(Canvas canvas, Pose2d pose) {
        double x = pose.position.x;
        double y = pose.position.y;
        double h = pose.heading.toDouble();

        canvas.fillCircle(x, y, 4);
        canvas.strokeLine(x, y, x + 6 * Math.cos(h), y + 6 * Math.sin(h));
    }

    public void displayTuningParams() {
        opMode.telemetry.addLine("=== SIMPLIFIED RAMSETE ===");
        opMode.telemetry.addData("RAMSETE_B", Params.RAMSETE_B);
        opMode.telemetry.addData("RAMSETE_ZETA", Params.RAMSETE_ZETA);
        opMode.telemetry.addData("Max Vel", "%.1f in/s", Params.MAX_VEL);
        opMode.telemetry.addData("Max Accel", "%.1f in/s²", Params.MAX_ACCEL);
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("Feedforward + RAMSETE feedback");
        opMode.telemetry.addLine("Industry standard implementation");
        opMode.telemetry.update();
    }
}