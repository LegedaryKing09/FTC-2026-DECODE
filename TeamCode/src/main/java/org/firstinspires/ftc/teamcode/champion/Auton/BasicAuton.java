package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

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

    // Autonomous parameters
    public static double SHOOTER_START_RPM = 2750.0;
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double MOVEMENT_SPEED = 0.3; // Conservative speed for odometry

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LimelightAlignmentController tempLimelight = null;
        try {
            // FIX: Pass both opMode (this) and driveController
            tempLimelight = new LimelightAlignmentController(this, driveController);
            tempLimelight.setTargetTag(AutoShootController.APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }
        limelightController = tempLimelight;

        // Initialize enhanced auto shoot controller with ramp controller (7 parameters now)
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
        telemetry.update();

        waitForStart();

        // STEP 1: Start shooter at specified RPM
        telemetry.addLine("=== STARTING SHOOTER ===");
        telemetry.addData("Target RPM", SHOOTER_START_RPM);
        telemetry.update();

        shooterController.setShooterRPM(SHOOTER_START_RPM);

        // Let shooter spin up for a moment
        sleep(1000);

        // STEP 2: Move backward using odometry
        telemetry.addLine("=== MOVING BACKWARD ===");
        telemetry.addData("Distance", BACKWARD_DISTANCE_INCHES + " inches");
        telemetry.update();

        moveBackwardWithOdometry(BACKWARD_DISTANCE_INCHES);

        // STEP 3: Execute auto-shoot sequence
        telemetry.addLine("=== EXECUTING AUTO-SHOOT ===");
        telemetry.update();

        autoShootController.executeDistanceBasedAutoShoot();

        // Wait for auto-shoot to complete
        while (opModeIsActive() && autoShootController.isAutoShooting()) {
            shooterController.updatePID();
            driveController.updateOdometry();
            telemetry.addLine("Auto-shooting in progress...");
            autoShootController.addTelemetry(telemetry);
            telemetry.update();
            sleep(50);
        }

        // STEP 4: Stop all systems
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        telemetry.addLine("=== AUTON COMPLETE ===");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Move backward a specified distance using odometry
     */
    private void moveBackwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startY = driveController.getY();
        double targetY = startY - distanceInches; // Negative Y is backward

        telemetry.addData("Starting Y", String.format("%.2f in", startY));
        telemetry.addData("Target Y", String.format("%.2f in", targetY));

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move backward at constant speed
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentY = driveController.getY();
            double distanceMoved = startY - currentY;

            telemetry.addData("Current Y", String.format("%.2f in", currentY));
            telemetry.addData("Distance Moved", String.format("%.2f in", distanceMoved));

            // Check if we've reached the target distance
            if (distanceMoved >= distanceInches) {
                break;
            }

            telemetry.update();
            sleep(20);
        }

        // Stop movement
        driveController.stopDrive();

        telemetry.addLine("Backward movement complete");
        telemetry.update();
        sleep(500);
    }
}