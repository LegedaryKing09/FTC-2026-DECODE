package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;

import java.util.Locale;

/**
 * Basic Autonomous sequence using odometry with Pinpoint
 * Sequence:
 * 1. Move backward 60in with shooter at half RPM
 * 2. Execute autoshoot sequence
 * 3. Turn left 90 degrees
 * 4. Move forward 20in with intake on
 * 5. Turn intake off and move backward 20in
 * 6. Turn right 90 degrees and execute autoshoot sequence again
 */
@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicAuton extends LinearOpMode {

    // Controllers
    private SixWheelDriveController driveController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private TransferController transferController;
    private RampController rampController;
    private AutoShootController autoShootController;
    private LimelightAlignmentController limelightController;

    // Telemetry
    private MultipleTelemetry multiTelemetry;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Display initialization status
        multiTelemetry.addLine("=== INITIALIZING BASIC AUTON ===");
        multiTelemetry.addLine("Initializing controllers...");
        multiTelemetry.update();

        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
        driveController.resetOdometry();

        // Initialize other controllers
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        transferController = new TransferController(this);
        rampController = new RampController(this);

        // Initialize Limelight controller (required for AutoShootController)
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);

            // Initialize AutoShootController
            autoShootController = new AutoShootController(
                    this,
                    driveController,
                    shooterController,
                    intakeController,
                    transferController,
                    limelightController,
                    rampController
            );

            multiTelemetry.addLine("✓ All controllers initialized successfully");
        } catch (Exception e) {
            multiTelemetry.addLine("⚠️ Warning: Limelight initialization failed:");
            multiTelemetry.addLine(e.getMessage());
            multiTelemetry.addLine("AutoShootController will not be available");
        }

        multiTelemetry.addLine("=== READY TO START ===");
        multiTelemetry.addLine("Press START to begin autonomous sequence");
        multiTelemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Sequence complete
        multiTelemetry.clear();
        multiTelemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        multiTelemetry.addData("Total Time", "%.2f seconds", runtime.seconds());
        multiTelemetry.update();

        // Keep telemetry active until stop requested
        while (opModeIsActive() && !isStopRequested()) {
            sleep(50);
        }
    }

    /**
     * Execute the complete autonomous sequence
     */
    private void executeAutonomousSequence() throws InterruptedException {
        multiTelemetry.clear();
        multiTelemetry.addLine("=== EXECUTING AUTONOMOUS SEQUENCE ===");
        multiTelemetry.update();

        // STEP 1: Move backward 60in with shooter at half RPM
        multiTelemetry.addLine("Step 1: Moving backward 60in with shooter at half RPM");
        multiTelemetry.update();
        executeStep1();

        // STEP 2: Execute autoshoot sequence
        multiTelemetry.addLine("Step 2: Executing autoshoot sequence");
        multiTelemetry.update();
        executeStep2();

        // STEP 3: Turn left 90 degrees
        multiTelemetry.addLine("Step 3: Turning left 90 degrees");
        multiTelemetry.update();
        executeStep3();

        // STEP 4: Move forward 20in with intake on
        multiTelemetry.addLine("Step 4: Moving forward 20in with intake on");
        multiTelemetry.update();
        executeStep4();

        // STEP 5: Turn intake off and move backward 20in
        multiTelemetry.addLine("Step 5: Turning intake off and moving backward 20in");
        multiTelemetry.update();
        executeStep5();

        // STEP 6: Turn right 90 degrees and execute autoshoot sequence again
        multiTelemetry.addLine("Step 6: Turning right 90 degrees and autoshooting again");
        multiTelemetry.update();
        executeStep6();
    }

    /**
     * Step 1: Move backward 60in with shooter at half RPM
     */
    private void executeStep1() throws InterruptedException {
        // Start shooter at half RPM FIRST
        shooterController.shooterHalf();

        // Let shooter start accelerating during movement
        // Update PID regularly to accelerate shooter
        shooterController.updatePID();

        // Get initial position
        driveController.updateOdometry();
        double startX = driveController.getX();

        // Move backward (negative Y direction) until 60 inches traveled
        double targetDistance = 60.0; // inches
        double speed = -0.3; // backward speed

        while (opModeIsActive() && Math.abs(driveController.getX() - startX) < targetDistance) {
            // Update odometry
            driveController.updateOdometry();

            // Keep shooter accelerating
            shooterController.updatePID();

            // Arcade drive backward (drive = negative, turn = 0)
            driveController.arcadeDrive(speed, 0);

            // Telemetry
            multiTelemetry.addData("Step 1 Progress", "%.1f / %.1f inches",
                    Math.abs(driveController.getX() - startX), targetDistance);
            multiTelemetry.addData("Current X", "%.2f", driveController.getX());
            multiTelemetry.addData("Shooter RPM", "%.0f", shooterController.getShooterRPM());
            multiTelemetry.addData("Target RPM", "%.0f", shooterController.getTargetRPM());
            multiTelemetry.update();

            sleep(20);
        }

        // Stop drive
        driveController.stopDrive();

        multiTelemetry.addLine("✓ Step 1 complete: Moved backward 60 inches");
        multiTelemetry.update();
        sleep(500);
    }

    /**
     * Step 2: Execute autoshoot sequence
     */
    private void executeStep2() throws InterruptedException {
        if (autoShootController != null) {
            // Execute autoshoot sequence
            autoShootController.executeDistanceBasedAutoShoot();

            if (!autoShootController.isAutoShooting()) {
                multiTelemetry.addLine("✓ Step 2 complete: Autoshoot sequence finished");
            } else {
                multiTelemetry.addLine("⚠️ Step 2 timeout: Autoshoot sequence did not complete");
            }
        } else {
            multiTelemetry.addLine("⚠️ Step 2 skipped: AutoShootController not available");
        }

        multiTelemetry.update();
        sleep(500);
    }

    /**
     * Step 3: Turn left 90 degrees
     */
    private void executeStep3() throws InterruptedException {
        // Use odometry IMU heading (from pinpoint) as used in tests and purepursuit
        driveController.updateOdometry();
        double startHeading = driveController.getHeadingDegrees();
        double targetHeading = startHeading - 90.0; // Left turn = negative

        // Normalize target heading to [-180, 180]
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        // Turn left (negative angular velocity)
        double turnSpeed = -0.4; // radians/sec

        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();
            double headingError = targetHeading - currentHeading;

            // Normalize error to [-180, 180]
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            multiTelemetry.addData("Step 3 Progress", "%.1f / %.1f degrees",
                    Math.abs(currentHeading - startHeading), 90.0);
            multiTelemetry.addData("Current Heading", "%.2f°", currentHeading);
            multiTelemetry.addData("Target Heading", "%.2f°", targetHeading);
            multiTelemetry.addData("Heading Error", "%.2f°", headingError);
            multiTelemetry.update();

            // Check if turn is complete (within 2 degrees)
            if (Math.abs(headingError) <= 10.0) {
                break;
            }

            // Apply turn using odometry heading
            driveController.arcadeDrive(0, turnSpeed);

            sleep(20);
        }

        // Stop drive
        driveController.stopDrive();

        multiTelemetry.addLine("✓ Step 3 complete: Turned left 90 degrees");
        multiTelemetry.update();
        sleep(500);
    }

    /**
     * Step 4: Move forward 20in with intake on
     */
    private void executeStep4() throws InterruptedException {
        // Start intake
        intakeController.intakeFull();

        // Get initial position
        driveController.updateOdometry();
        double startY = driveController.getY();

        // Move forward (positive Y direction) until 20 inches traveled
        double targetDistance = 20.0; // inches
        double speed = 0.3; // forward speed

        while (opModeIsActive() && Math.abs(driveController.getY() - startY) < targetDistance) {
            // Update odometry
            driveController.updateOdometry();

            // Arcade drive forward (drive = positive, turn = 0)
            driveController.arcadeDrive(speed, 0);

            // Telemetry
            multiTelemetry.addData("Step 4 Progress", "%.1f / %.1f inches",
                    Math.abs(driveController.getY() - startY), targetDistance);
            multiTelemetry.addData("Current Y", "%.2f", driveController.getY());
            multiTelemetry.addData("Intake Power", "%.2f", intakeController.getIntakePower());
            multiTelemetry.update();

            sleep(20);
        }

        // Stop drive
        driveController.stopDrive();

        multiTelemetry.addLine("✓ Step 4 complete: Moved forward 20 inches with intake on");
        multiTelemetry.update();
        sleep(500);
    }

    /**
     * Step 5: Turn intake off and move backward 20in
     */
    private void executeStep5() throws InterruptedException {
        // Turn intake off
        intakeController.intakeStop();

        // Get initial position
        driveController.updateOdometry();
        double startY = driveController.getY();

        // Move backward (negative Y direction) until 20 inches traveled
        double targetDistance = 20.0; // inches
        double speed = -0.3; // backward speed

        while (opModeIsActive() && Math.abs(driveController.getY() - startY) < targetDistance) {
            // Update odometry
            driveController.updateOdometry();

            // Arcade drive backward (drive = negative, turn = 0)
            driveController.arcadeDrive(speed, 0);

            // Telemetry
            multiTelemetry.addData("Step 5 Progress", "%.1f / %.1f inches",
                    Math.abs(driveController.getY() - startY), targetDistance);
            multiTelemetry.addData("Current Y", "%.2f", driveController.getY());
            multiTelemetry.addData("Intake Power", "%.2f", intakeController.getIntakePower());
            multiTelemetry.update();

            sleep(20);
        }

        // Stop drive
        driveController.stopDrive();

        multiTelemetry.addLine("✓ Step 5 complete: Intake off, moved backward 20 inches");
        multiTelemetry.update();
        sleep(500);
    }

    /**
     * Step 6: Turn right 90 degrees and execute autoshoot sequence again
     */
    private void executeStep6() throws InterruptedException {
        // Turn right 90 degrees - using odometry IMU heading
        driveController.updateOdometry();
        double startHeading = driveController.getHeadingDegrees();
        double targetHeading = startHeading + 90.0; // Right turn = positive

        // Normalize target heading to [-180, 180]
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        // Turn right (positive angular velocity)
        double turnSpeed = 0.4; // radians/sec

        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();
            double headingError = targetHeading - currentHeading;

            // Normalize error to [-180, 180]
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            multiTelemetry.addData("Step 6a Progress", "%.1f / %.1f degrees",
                    Math.abs(currentHeading - startHeading), 90.0);
            multiTelemetry.addData("Current Heading", "%.2f°", currentHeading);
            multiTelemetry.addData("Target Heading", "%.2f°", targetHeading);
            multiTelemetry.addData("Heading Error", "%.2f°", headingError);
            multiTelemetry.update();

            // Check if turn is complete (within 2 degrees)
            if (Math.abs(headingError) <= 10.0) {
                break;
            }

            // Apply turn using odometry heading
            driveController.arcadeDrive(0, turnSpeed);

            sleep(20);
        }

        // Stop drive
        driveController.stopDrive();

        multiTelemetry.addLine("✓ Step 6a complete: Turned right 90 degrees");
        multiTelemetry.update();
        sleep(500);

        // Execute autoshoot sequence again - shooter should already be at half RPM
        multiTelemetry.addLine("Step 6b: Executing autoshoot sequence again");
        multiTelemetry.update();

        if (autoShootController != null) {
            // Execute autoshoot sequence - this will change RPM to calculated value, then back to half after
            autoShootController.executeDistanceBasedAutoShoot();

            if (!autoShootController.isAutoShooting()) {
                multiTelemetry.addLine("✓ Step 6 complete: Second autoshoot sequence finished");
            } else {
                multiTelemetry.addLine("⚠️ Step 6 timeout: Second autoshoot sequence did not complete");
            }
        } else {
            multiTelemetry.addLine("⚠️ Step 6b skipped: AutoShootController not available");
        }

        multiTelemetry.update();
        sleep(500);
    }
}