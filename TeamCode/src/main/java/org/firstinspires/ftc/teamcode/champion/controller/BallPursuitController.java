package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Ball Pursuit Controller - Combines ball alignment with pure pursuit pathing
 * Robot aligns to ball and follows it dynamically as it moves
 */
@Config
public class BallPursuitController {

    private final LinearOpMode opMode;
    private final SimpleBallAlignmentController ballAlignment;
    private final PurePursuitController pursuitController;
    private final SixWheelDriveController driveController;

    @Config
    public static class BallPursuitConfig {
        public static double APPROACH_DISTANCE = 12.0;        // Distance to approach ball (inches)
        public static double STOP_DISTANCE = 4.0;             // Stop when this close to ball
        public static double BALL_UPDATE_DISTANCE = 6.0;      // Min distance ball must move to update path
        public static double MAX_SPEED = 0.6;                 // Maximum pursuit speed
        public static double MIN_SPEED = 0.2;                 // Minimum pursuit speed
        public static double PATH_UPDATE_INTERVAL = 0.5;      // Seconds between path updates
        public static double BALL_LOST_TIMEOUT = 2.0;         // Stop if ball lost for this long
        public static double HEADING_CORRECTION_WEIGHT = 0.7; // How much to prioritize heading correction
    }

    // State tracking - simplified to avoid state machine issues
    private boolean isActive = false;
    private Vector2d lastBallPosition = null;
    private Vector2d currentTargetPosition = null;
    private final ElapsedTime pathUpdateTimer = new ElapsedTime();
    private final ElapsedTime ballLostTimer = new ElapsedTime();
    private boolean ballWasLost = false;
    private boolean isPursuing = false; // Simple flag instead of enum state

    // Removed enum to avoid state machine issues in FTC

    public BallPursuitController(LinearOpMode opMode) throws Exception {
        this.opMode = opMode;

        // Initialize controllers
        this.driveController = new SixWheelDriveController(opMode);
        this.ballAlignment = new SimpleBallAlignmentController(opMode);
        this.pursuitController = new PurePursuitController();

        // Configure pursuit controller
        pursuitController.setParameters(12.0, BallPursuitConfig.MAX_SPEED, 11.0);
    }

    /**
     * Start pursuing the ball
     */
    public void startPursuit() {
        isActive = true;
        isPursuing = false;  // Start as not pursuing, let update logic handle state
        ballAlignment.startTracking();
        pathUpdateTimer.reset();
        ballLostTimer.reset();
        ballWasLost = false;
        lastBallPosition = null;
        currentTargetPosition = null;

        // Safety: Ensure we're not moving too fast initially
        BallPursuitConfig.MAX_SPEED = Math.min(BallPursuitConfig.MAX_SPEED, 0.5);
    }

    /**
     * Stop pursuit
     */
    public void stopPursuit() {
        isActive = false;
        isPursuing = false;
        ballAlignment.stopTracking();
        pursuitController.setPath(null);
        driveController.stopDrive();
        lastBallPosition = null;
        currentTargetPosition = null;
    }

    /**
     * Main pursuit update loop - call this repeatedly
     */
    public void updatePursuit() {
        if (!isActive) return;

        // Update odometry for pursuit controller
        driveController.updateOdometry();

        // Update ball alignment
        ballAlignment.align();

        // Get current robot pose
        Pose2d currentPose = new Pose2d(
            driveController.getX(),
            driveController.getY(),
            driveController.getHeading()
        );

        // Simplified pursuit logic without state machine
        boolean hasBall = ballAlignment.hasBall();
        Vector2d ballPosition = calculateBallPosition(currentPose);

        // Handle ball detection/loss
    if (hasBall) {
        ballLostTimer.reset();
        ballWasLost = false;

        // Check if close enough to ball
        double distanceToBall = currentPose.position.minus(ballPosition).norm();
        if (distanceToBall <= BallPursuitConfig.STOP_DISTANCE) {
            // At ball - stop
            isPursuing = false;
            driveController.stopDrive();
            return;
        }

        // Check if ball moved significantly enough to update path
        boolean shouldUpdatePath = shouldUpdatePath(ballPosition);

        if (shouldUpdatePath) {
            currentTargetPosition = calculateTargetPosition(currentPose, ballPosition);
            pursuitController.setTargetPosition(currentTargetPosition);
            lastBallPosition = ballPosition;
            pathUpdateTimer.reset();
            isPursuing = true;
        }

        // If pursuing, combine pure pursuit with alignment
        if (isPursuing) {
            // Get pursuit powers
            double[] pursuitPowers = pursuitController.update(currentPose);

            // Get alignment correction (heading adjustment)
            double alignmentCorrection = getAlignmentCorrection();

            // Combine pursuit and alignment
            double leftPower = pursuitPowers[0] + (alignmentCorrection * BallPursuitConfig.HEADING_CORRECTION_WEIGHT);
            double rightPower = pursuitPowers[1] - (alignmentCorrection * BallPursuitConfig.HEADING_CORRECTION_WEIGHT);

            // Safety: Clamp powers to prevent excessive speed
            leftPower = Math.max(-BallPursuitConfig.MAX_SPEED, Math.min(BallPursuitConfig.MAX_SPEED, leftPower));
            rightPower = Math.max(-BallPursuitConfig.MAX_SPEED, Math.min(BallPursuitConfig.MAX_SPEED, rightPower));

            // Apply minimum speed if moving but very slow
            if (Math.abs(leftPower) > 0.05 && Math.abs(leftPower) < BallPursuitConfig.MIN_SPEED) {
                leftPower = Math.signum(leftPower) * BallPursuitConfig.MIN_SPEED;
            }
            if (Math.abs(rightPower) > 0.05 && Math.abs(rightPower) < BallPursuitConfig.MIN_SPEED) {
                rightPower = Math.signum(rightPower) * BallPursuitConfig.MIN_SPEED;
            }

            // Apply powers
            driveController.tankDrive(leftPower, rightPower);
        }
    } else {
        // Ball lost
        if (!ballWasLost) {
            ballLostTimer.reset();
            ballWasLost = true;
        }

        if (ballLostTimer.seconds() > BallPursuitConfig.BALL_LOST_TIMEOUT) {
            // Ball lost for too long - stop
            isPursuing = false;
            driveController.stopDrive();
            return;
        }

        // Keep trying to find ball - ball alignment controller handles searching
        isPursuing = false;
    }
    }


    /**
     * Calculate field-absolute ball position from Limelight data
     * Handles multiple balls by finding the closest one
     */
    private Vector2d calculateBallPosition(Pose2d robotPose) {
        if (!ballAlignment.hasBall()) {
            return null;
        }

        double ballDistance = ballAlignment.getBallDistance();
        double ballAngle = ballAlignment.getTx(); // TX is horizontal angle to ball

        // Safety: Validate distance
        if (ballDistance <= 0 || ballDistance > 200) { // Reasonable bounds for FTC field
            return null;
        }

        // Convert polar coordinates to Cartesian relative to robot
        double ballXRelative = ballDistance * Math.cos(Math.toRadians(ballAngle));
        double ballYRelative = ballDistance * Math.sin(Math.toRadians(ballAngle));

        // Transform to field coordinates
        double robotX = robotPose.position.x;
        double robotY = robotPose.position.y;
        double robotHeading = robotPose.heading.toDouble();

        double ballXField = robotX + ballXRelative * Math.cos(robotHeading) - ballYRelative * Math.sin(robotHeading);
        double ballYField = robotY + ballXRelative * Math.sin(robotHeading) + ballYRelative * Math.cos(robotHeading);

        return new Vector2d(ballXField, ballYField);
    }

    /**
     * Calculate target position for pure pursuit (approach but don't collide)
     */
    private Vector2d calculateTargetPosition(Pose2d robotPose, Vector2d ballPosition) {
        if (ballPosition == null) return null;

        // Vector from robot to ball
        Vector2d toBall = ballPosition.minus(robotPose.position);
        double distanceToBall = toBall.norm();

        if (distanceToBall <= BallPursuitConfig.APPROACH_DISTANCE) {
            // Close enough, target the ball itself
            return ballPosition;
        } else {
            // Target a point along the line to the ball
            Vector2d direction = toBall.div(distanceToBall);
            return ballPosition.minus(direction.times(BallPursuitConfig.APPROACH_DISTANCE));
        }
    }

    /**
     * Check if ball has moved enough to warrant path update
     */
    private boolean shouldUpdatePath(Vector2d newBallPosition) {
        if (lastBallPosition == null || newBallPosition == null) {
            return true;
        }

        double distanceMoved = lastBallPosition.minus(newBallPosition).norm();
        boolean timeToUpdate = pathUpdateTimer.seconds() > BallPursuitConfig.PATH_UPDATE_INTERVAL;

        return distanceMoved > BallPursuitConfig.BALL_UPDATE_DISTANCE || timeToUpdate;
    }

    /**
     * Get heading correction from ball alignment controller
     */
    private double getAlignmentCorrection() {
        return ballAlignment.getMotorOutput();
    }

    // Getters for telemetry and debugging
    public String getState() {
        if (!isActive) return "STOPPED";
        if (ballWasLost && ballLostTimer.seconds() > BallPursuitConfig.BALL_LOST_TIMEOUT) return "BALL_LOST";
        if (!hasBall()) return "SEARCHING";
        if (isPursuing) return "PURSUING";
        return "AT_BALL";
    }

    public boolean isPursuing() {
        return isActive && isPursuing;
    }

    public boolean hasBall() {
        return ballAlignment.hasBall();
    }

    public double getDistanceToBall() {
        if (!hasBall()) return Double.MAX_VALUE;

        Pose2d currentPose = new Pose2d(
            driveController.getX(),
            driveController.getY(),
            driveController.getHeading()
        );

        Vector2d ballPosition = calculateBallPosition(currentPose);
        if (ballPosition == null) return Double.MAX_VALUE;

        return currentPose.position.minus(ballPosition).norm();
    }

    public Vector2d getBallPosition() {
        Pose2d currentPose = new Pose2d(
            driveController.getX(),
            driveController.getY(),
            driveController.getHeading()
        );
        return calculateBallPosition(currentPose);
    }

    public Vector2d getTargetPosition() {
        return currentTargetPosition;
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("╔═══ BALL PURSUIT ═══╗");
        opMode.telemetry.addData("State", getState());
        opMode.telemetry.addData("Has Ball", hasBall());
        opMode.telemetry.addData("Pursuing", isPursuing());

        if (hasBall()) {
            Vector2d ballPos = getBallPosition();
            if (ballPos != null) {
                opMode.telemetry.addData("Ball Pos", "%.1f, %.1f", ballPos.x, ballPos.y);
                opMode.telemetry.addData("Distance", "%.1f in", getDistanceToBall());
            }

            if (currentTargetPosition != null) {
                opMode.telemetry.addData("Target", "%.1f, %.1f", currentTargetPosition.x, currentTargetPosition.y);
            }
        }

        ballAlignment.displayTelemetry();
        opMode.telemetry.addLine("╚══════════════════╝");
    }
}