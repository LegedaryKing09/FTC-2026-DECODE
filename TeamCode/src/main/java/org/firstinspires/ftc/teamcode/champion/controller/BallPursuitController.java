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

    // State tracking
    private boolean isActive = false;
    private Vector2d lastBallPosition = null;
    private Vector2d currentTargetPosition = null;
    private final ElapsedTime pathUpdateTimer = new ElapsedTime();
    private final ElapsedTime ballLostTimer = new ElapsedTime();
    private boolean ballWasLost = false;

    public enum PursuitState {
        STOPPED,
        ALIGNING_TO_BALL,
        PURSUING_BALL,
        AT_BALL,
        BALL_LOST
    }

    private PursuitState currentState = PursuitState.STOPPED;

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
        currentState = PursuitState.ALIGNING_TO_BALL;
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
        currentState = PursuitState.STOPPED;
        ballAlignment.stopTracking();
        pursuitController.setPath(null);
        driveController.stopDrive();
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

        // Determine pursuit state and control robot
        updatePursuitState(currentPose);
    }

    private void updatePursuitState(Pose2d currentPose) {
        boolean hasBall = ballAlignment.hasBall();
        Vector2d ballPosition = calculateBallPosition(currentPose);

        // Handle ball detection/loss
        if (hasBall) {
            ballLostTimer.reset();
            ballWasLost = false;

            // Check if ball moved significantly enough to update path
            boolean shouldUpdatePath = shouldUpdatePath(ballPosition);

            if (shouldUpdatePath) {
                currentTargetPosition = calculateTargetPosition(currentPose, ballPosition);
                pursuitController.setTargetPosition(currentTargetPosition);
                lastBallPosition = ballPosition;
                pathUpdateTimer.reset();
            }
        } else {
            // Ball lost
            if (!ballWasLost) {
                ballLostTimer.reset();
                ballWasLost = true;
            }

            if (ballLostTimer.seconds() > BallPursuitConfig.BALL_LOST_TIMEOUT) {
                currentState = PursuitState.BALL_LOST;
                driveController.stopDrive();
                return;
            }
        }

        // State machine
        switch (currentState) {
            case ALIGNING_TO_BALL:
                handleAligningState(currentPose, hasBall);
                break;

            case PURSUING_BALL:
                handlePursuingState(currentPose, hasBall);
                break;

            case AT_BALL:
                handleAtBallState();
                break;

            case BALL_LOST:
                handleBallLostState();
                break;

            case STOPPED:
                break;
        }
    }

    private void handleAligningState(Pose2d currentPose, boolean hasBall) {
        if (!hasBall) {
            currentState = PursuitState.BALL_LOST;
            return;
        }

        // Check if aligned to ball
        if (ballAlignment.isAligned()) {
            currentState = PursuitState.PURSUING_BALL;
            // Ball alignment controller is still running to maintain heading
        }
        // Ball alignment controller handles the turning to face the ball
    }

    private void handlePursuingState(Pose2d currentPose, boolean hasBall) {
        if (!hasBall) {
            currentState = PursuitState.BALL_LOST;
            return;
        }

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

        // Check if close enough to ball
        Vector2d ballPosition = calculateBallPosition(currentPose);
        double distanceToBall = currentPose.position.minus(ballPosition).norm();

        if (distanceToBall <= BallPursuitConfig.STOP_DISTANCE) {
            currentState = PursuitState.AT_BALL;
            driveController.stopDrive();
        }
    }

    private void handleAtBallState() {
        // Stay stopped at the ball
        driveController.stopDrive();

        // If ball moves away, start pursuing again
        if (ballAlignment.hasBall()) {
            Vector2d ballPosition = calculateBallPosition(new Pose2d(
                driveController.getX(), driveController.getY(), driveController.getHeading()));

            if (ballPosition != null) {
                double distanceToBall = Math.hypot(
                    driveController.getX() - ballPosition.x,
                    driveController.getY() - ballPosition.y);

                if (distanceToBall > BallPursuitConfig.STOP_DISTANCE + 2.0) {
                    currentState = PursuitState.PURSUING_BALL;
                }
            }
        }
    }

    private void handleBallLostState() {
        // Ball alignment will handle searching
        // Just wait for ball to be found again
        if (ballAlignment.hasBall()) {
            currentState = PursuitState.ALIGNING_TO_BALL;
        }
    }

    /**
     * Calculate field-absolute ball position from Limelight data
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
    public PursuitState getState() {
        return currentState;
    }

    public boolean isPursuing() {
        return isActive;
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
        opMode.telemetry.addData("State", currentState);
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