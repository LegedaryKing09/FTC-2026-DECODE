package org.firstinspires.ftc.teamcode.champion.Auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.PurePursuitController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

import java.util.Locale;

@Config
@Autonomous(name = "Basic Auton", group = "Competition")
public class BasicAuton extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;
    PurePursuitController pursuitController;

    // Autonomous parameters
    public static double SHOOTER_START_RPM = 2750.0;
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double FORWARD_DISTANCE_INCHES = 3.0;
    public static double REPOSITIONING_DISTANCE = 3.0;
    public static double MOVEMENT_SPEED = 0.3; // Conservative speed for odometry
    public static long AUTO_SHOOT_TIMEOUT = 8000; // Maximum time to wait for auto-shoot sequence
    public static long SETTLE_TIME = 500; // Time to settle after movement before shooting
    public static double TURN_ANGLE_DEGREES = 30.0; // Angle to turn for repositioning

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);
        pursuitController = new PurePursuitController();
        pursuitController.setParameters(4.0, 0.6, 11.0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        LimelightAlignmentController tempLimelight = null;
        try {
            tempLimelight = new LimelightAlignmentController(this, driveController);
            tempLimelight.setTargetTag(AutoShootController.APRILTAG_ID);
            telemetry.addData("Limelight", "✅ Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
        }
        limelightController = tempLimelight;

        // Initialize auto shoot controller with all required dependencies
        autoShootController = new AutoShootController(
                this,
                driveController,
                shooterController,
                intakeController,
                transferController,
                limelightController,
                rampController
        );

        // Initialize ramp to starting position
        rampController.setTo0Degrees();

        telemetry.addLine("=== BASIC AUTON READY ===");
        telemetry.addData("Shooter Start RPM", SHOOTER_START_RPM);
        telemetry.addData("Backward Distance", BACKWARD_DISTANCE_INCHES + " inches");
        telemetry.addData("AprilTag Target", AutoShootController.APRILTAG_ID);
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // Start shooter spinning early
        telemetry.addLine("Starting shooter...");
        telemetry.update();
        shooterController.setShooterRPM(SHOOTER_START_RPM);

        // Execute backward movement
        telemetry.addLine("Moving backward...");
        telemetry.update();
        moveBackwardWithOdometry(BACKWARD_DISTANCE_INCHES);

        // Settle after movement
        telemetry.addLine("Settling...");
        telemetry.update();
        sleep(SETTLE_TIME);

        // Execute auto-shoot sequence - FIRST SHOT
        telemetry.addLine("Starting first auto-shoot sequence...");
        telemetry.update();
        executeAutoShootSequence();

        // Wait for first sequence to fully complete
        sleep(500);

        // Start intake for second ball
        telemetry.addLine("Starting intake for second ball...");
        telemetry.update();
        intakeController.intakeFull();

        // Reposition for second shot
        turnToHeading(TURN_ANGLE_DEGREES);
        moveForwardWithOdometry(FORWARD_DISTANCE_INCHES);
        moveBackwardWithOdometry(REPOSITIONING_DISTANCE);
        turnToHeading(0);

        // Stop intake before second shot
        intakeController.intakeStop();
        sleep(200);

        // Execute auto-shoot sequence - SECOND SHOT
        telemetry.addLine("Starting second auto-shoot sequence...");
        telemetry.update();
        executeAutoShootSequence();

        // Final status
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
        telemetry.update();
    }

    /**
     * Move forward a specified distance using odometry
     */
    @SuppressLint("DefaultLocale")
    private void moveForwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX + distanceInches; // Positive X is forward

        telemetry.addData("Starting X", String.format(Locale.US, "%.2f in", startX));
        telemetry.addData("Target X", String.format(Locale.US, "%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move forward at constant speed
        driveController.tankDriveVelocityNormalized(MOVEMENT_SPEED, MOVEMENT_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(currentX - startX);

            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format(Locale.US, "%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", distanceInches));
            telemetry.update();

            // Check if we've reached the target distance
            if (distanceMoved >= Math.abs(distanceInches)) {
                break;
            }

            sleep(20);
        }

        // Stop movement
        driveController.stopDrive();

        telemetry.addLine("✅ Forward movement complete");
        telemetry.update();
        sleep(200);
    }

    /**
     * Move backward a specified distance using odometry
     */
    @SuppressLint("DefaultLocale")
    private void moveBackwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX - distanceInches; // Negative X is backward

        telemetry.addData("Starting X", String.format(Locale.US, "%.2f in", startX));
        telemetry.addData("Target X", String.format(Locale.US, "%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move backward at constant speed
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(startX - currentX);

            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format(Locale.US, "%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", distanceInches));
            telemetry.update();

            // Check if we've reached the target distance
            if (distanceMoved >= Math.abs(distanceInches)) {
                break;
            }

            sleep(20);
        }

        // Stop movement
        driveController.stopDrive();

        telemetry.addLine("✅ Backward movement complete");
        telemetry.update();
        sleep(200);
    }

    /**
     * Execute the auto-shoot sequence and wait for completion
     * Fixed to match AutoTeleop.java working implementation
     */
    @SuppressLint("DefaultLocale")
    private void executeAutoShootSequence() {
        // Trigger the auto-shoot sequence (runs in separate thread)
        // This method handles alignment, shooter speed, ramp angle, and transfer automatically
        autoShootController.executeDistanceBasedAutoShoot();

        // Wait for auto-shoot to complete or timeout
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && autoShootController.isAutoShooting() &&
                timer.milliseconds() < AUTO_SHOOT_TIMEOUT) {

            // Update shooter PID during the sequence
            // This is critical for maintaining target RPM
            shooterController.updatePID();

            // Display status
            telemetry.addLine("=== AUTO-SHOOT IN PROGRESS ===");
            telemetry.addData("Status", autoShootController.getCurrentStatus());
            telemetry.addData("Time Elapsed", String.format(Locale.US, "%.1f s", timer.seconds()));
            telemetry.addData("Timeout", String.format(Locale.US, "%.1f s", AUTO_SHOOT_TIMEOUT / 1000.0));

            // Show shooter status
            telemetry.addData("Current RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
            telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", shooterController.getTargetRPM()));
            telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✅ YES" : "NO");

            // Show ramp status
            telemetry.addData("Ramp Angle", String.format(Locale.US, "%.1f°", rampController.getAngle()));

            // Show alignment status if limelight is available
            if (limelightController != null && limelightController.hasTarget()) {
                telemetry.addData("Alignment Error", String.format(Locale.US, "%.2f°", limelightController.getTargetError()));
            }

            telemetry.update();
            sleep(50);
        }

        // Check if timeout occurred
        if (timer.milliseconds() >= AUTO_SHOOT_TIMEOUT) {
            telemetry.addLine("⚠️ Auto-shoot timeout - continuing...");
            telemetry.update();
            sleep(1000);
        }

        // Give a brief moment for the sequence to fully complete
        sleep(200);

        // Clean stop of all systems (the AutoShootController should have already done this)
        // But we do it again to be safe
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        telemetry.addLine("✅ Auto-shoot sequence complete");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    private void goToPositionWithPursuit(double x, double y) {
        Vector2d targetPosition = new Vector2d(x, y);
        pursuitController.setTargetPosition(targetPosition);

        final double DISTANCE_THRESHOLD = 5.0;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            Pose2d currentPose = new Pose2d(
                    driveController.getX(),
                    driveController.getY(),
                    driveController.getHeading()
            );

            double distToEnd = Math.hypot(
                    currentPose.position.x - targetPosition.x,
                    currentPose.position.y - targetPosition.y
            );

            if (distToEnd < DISTANCE_THRESHOLD) {
                driveController.stopDrive();
                telemetry.addLine("✅ Position reached");
                telemetry.update();
                break;
            }

            double[] powers = pursuitController.update(currentPose);
            driveController.tankDrive(powers[0], powers[1]);

            telemetry.addData("Target", String.format(Locale.US, "(%.1f, %.1f)", x, y));
            telemetry.addData("Current", String.format(Locale.US, "(%.1f, %.1f)", currentPose.position.x, currentPose.position.y));
            telemetry.addData("Dist to Target", String.format(Locale.US, "%.1f in", distToEnd));
            telemetry.update();

            sleep(20);
        }
    }

    private void turnToHeading(double targetHeadingDeg) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        final double HEADING_THRESHOLD = Math.toRadians(2.0);
        final double TURN_POWER = 0.3;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = targetHeadingRad - currentHeading;

            // Normalize error to [-PI, PI]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            if (Math.abs(headingError) < HEADING_THRESHOLD) {
                driveController.stopDrive();
                telemetry.addLine("✅ Turn complete");
                telemetry.update();
                break;
            }

            double turnPower = Math.signum(headingError) * TURN_POWER;
            driveController.tankDrive(-turnPower, turnPower);

            telemetry.addData("Target Heading", String.format(Locale.US, "%.1f°", targetHeadingDeg));
            telemetry.addData("Current Heading", String.format(Locale.US, "%.1f°", Math.toDegrees(currentHeading)));
            telemetry.update();

            sleep(20);
        }
    }
}