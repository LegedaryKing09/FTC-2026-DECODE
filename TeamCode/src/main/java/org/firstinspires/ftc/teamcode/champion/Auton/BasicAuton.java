package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.champion.tests.PurePursuitTest;

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
    PurePursuitTest pathing;

    // Autonomous parameters
    public static double SHOOTER_START_RPM = 2750.0;
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double MOVEMENT_SPEED = 0.3; // Conservative speed for odometry
    public static long AUTO_SHOOT_TIMEOUT = 8000; // Maximum time to wait for auto-shoot sequence
    public static long SETTLE_TIME = 500; // Time to settle after movement before shooting

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);
        pursuitController = new PurePursuitController();
        pathing = new PurePursuitTest();
        pursuitController.setParameters(12.0, 0.5, 15.0);

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

        // Execute auto-shoot sequence
        telemetry.addLine("Starting auto-shoot sequence...");
        telemetry.update();
        executeAutoShootSequence();

        // Final status
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
        telemetry.update();
    }

    /**
     * Move backward a specified distance using odometry
     */
    private void moveBackwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX - distanceInches; // Negative X is backward

        telemetry.addData("Starting X", String.format("%.2f in", startX));
        telemetry.addData("Target X", String.format("%.2f in", targetX));
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

            telemetry.addData("Current X", String.format("%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format("%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format("%.2f in", distanceInches));
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
     */
    private void executeAutoShootSequence() {
        // Trigger the auto-shoot sequence (runs in separate thread)
        autoShootController.executeDistanceBasedAutoShoot();

        // Wait for auto-shoot to complete or timeout
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && autoShootController.isAutoShooting() &&
                timer.milliseconds() < AUTO_SHOOT_TIMEOUT) {

            // Update shooter PID during the sequence
            shooterController.updatePID();

            // Display status
            telemetry.addLine("=== AUTO-SHOOT IN PROGRESS ===");
            telemetry.addData("Status", autoShootController.getCurrentStatus());
            telemetry.addData("Time Elapsed", String.format("%.1f s", timer.seconds()));
            telemetry.addData("Timeout", String.format("%.1f s", AUTO_SHOOT_TIMEOUT / 1000.0));

            // Show shooter status
            telemetry.addData("Current RPM", String.format("%.0f", shooterController.getShooterRPM()));
            telemetry.addData("Target RPM", String.format("%.0f", shooterController.getTargetRPM()));
            telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✅ YES" : "NO");

            // Show ramp status
            telemetry.addData("Ramp Angle", String.format("%.1f°", rampController.getAngle()));

            telemetry.update();
            sleep(50);
        }

        // Check if timeout occurred
        if (timer.milliseconds() >= AUTO_SHOOT_TIMEOUT) {
            telemetry.addLine("⚠️ Auto-shoot timeout - continuing...");
            telemetry.update();
            sleep(1000);
        }

        // Stop all systems
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();
    }
}