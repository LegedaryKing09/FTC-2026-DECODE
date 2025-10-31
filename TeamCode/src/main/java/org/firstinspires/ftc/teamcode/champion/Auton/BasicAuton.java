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

    // Autonomous parameters - CONSTANT VALUES
    public static double CONSTANT_SHOOTER_RPM = 2800.0;  // Fixed RPM for entire autonomous
    public static double CONSTANT_RAMP_ANGLE = 120.0;    // Fixed ramp angle for entire autonomous
    public static double INTAKE_SPEED = 0.2;
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double FORWARD_DISTANCE_INCHES = 35;
    public static double REPOSITIONING_DISTANCE = 20;
    public static double MOVEMENT_SPEED = 0.3;
    public static long ALIGNMENT_TIMEOUT = 1000;
    public static long SHOOT_DURATION = 1500;
    public static long SETTLE_TIME = 500;
    public static double TURN_ANGLE_DEGREES = 21.0;
    public static double ALIGNMENT_THRESHOLD = 1.0;

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
        // We'll use it ONLY for telemetry and status tracking, NOT for the actual shooting logic
        autoShootController = new AutoShootController(
                this,
                driveController,
                shooterController,
                intakeController,
                transferController,
                limelightController,
                rampController
        );

        telemetry.addLine("=== BASIC AUTON READY ===");
        telemetry.addData("Constant Shooter RPM", CONSTANT_SHOOTER_RPM);
        telemetry.addData("Constant Ramp Angle", CONSTANT_RAMP_ANGLE);
        telemetry.addData("Backward Distance", BACKWARD_DISTANCE_INCHES + " inches");
        telemetry.addData("AprilTag Target", AutoShootController.APRILTAG_ID);
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // ===== START SHOOTER AND SET RAMP IMMEDIATELY =====
        telemetry.addLine("🚀 Starting shooter at constant RPM and setting ramp angle...");
        telemetry.addData("RPM", CONSTANT_SHOOTER_RPM);
        telemetry.addData("Ramp Angle", String.format(Locale.US, "%.1f°", CONSTANT_RAMP_ANGLE));
        telemetry.update();

        // Start shooter at constant RPM
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);

        // Set ramp to constant angle
        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        // Give the shooter and ramp a moment to start
        sleep(200);

        // Execute backward movement
        telemetry.addLine("Moving backward...");
        telemetry.update();
        moveBackwardWithOdometry(BACKWARD_DISTANCE_INCHES);

        // Settle after movement
        telemetry.addLine("Settling...");
        telemetry.update();
        sleep(SETTLE_TIME);

        // Execute FIRST SHOT with our custom sequence
        telemetry.addLine("Starting first shooting sequence...");
        telemetry.update();
        executeCustomShootSequence();

        // Wait for first shot to fully complete
        sleep(500);

        // Start intake for second ball
        telemetry.addLine("Starting intake for second ball...");
        telemetry.update();
        intakeController.intakeFull();

        // ===== IMPORTANT: SHOOTER KEEPS RUNNING DURING REPOSITIONING =====

        // Reposition for second shot
        turnToHeading(TURN_ANGLE_DEGREES);
        ForwardForIntakeWithShooterUpdate(FORWARD_DISTANCE_INCHES);
        moveBackwardWithOdometryAndShooterUpdate(REPOSITIONING_DISTANCE);
        turnToHeading(0);

        // Stop intake before second shot
        intakeController.intakeStop();
        sleep(200);

        // Execute SECOND SHOT with our custom sequence
        telemetry.addLine("Starting second shooting sequence...");
        telemetry.update();
        executeCustomShootSequence();

        // Final status
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Total Shots", "2");
        telemetry.update();

        // Only stop the shooter at the very end of autonomous
        sleep(500);
        shooterController.shooterStop();
    }

    /**
     * Custom shoot sequence that doesn't use AutoShootController's distance-based logic
     * Only handles alignment and firing, shooter RPM and ramp are already set
     */
    @SuppressLint("DefaultLocale")
    private void executeCustomShootSequence() {

        // STEP 1: Quick verification that shooter is at speed
        if (Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) > 150) {
            telemetry.addLine("⚠️ Waiting for shooter to stabilize...");
            telemetry.addData("Current RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
            telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", CONSTANT_SHOOTER_RPM));
            telemetry.update();

            // Brief wait for RPM to stabilize (max 2 seconds)
            ElapsedTime rpmTimer = new ElapsedTime();
            while (opModeIsActive() &&
                    Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) > 150 &&
                    rpmTimer.milliseconds() < 1000) {
                shooterController.updatePID();
                sleep(50);
            }
        }

        // STEP 2: Align to target using Limelight
        boolean alignmentAchieved = false;
        if (limelightController != null) {
            try {
                // Set drive controller to velocity mode for alignment
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                ElapsedTime alignTimer = new ElapsedTime();

                // Alignment loop
                while (opModeIsActive() && alignTimer.milliseconds() < ALIGNMENT_TIMEOUT) {
                    // Update shooter PID while aligning
                    shooterController.updatePID();

                    limelightController.align(AutoShootController.APRILTAG_ID);
                    double alignmentError = limelightController.getTargetError();

                    telemetry.addLine("🎯 Aligning to target...");
                    telemetry.addData("Error", String.format(Locale.US, "%.2f°", alignmentError));
                    telemetry.addData("Threshold", String.format(Locale.US, "%.2f°", ALIGNMENT_THRESHOLD));
                    telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
                    telemetry.update();

                    if (limelightController.isAligned()) {
                        alignmentAchieved = true;
                        break;
                    }

                    sleep(20);
                }

                // Stop alignment
                limelightController.stopAlignment();
                driveController.stopDrive();

                if (!alignmentAchieved) {
                    telemetry.addLine("⚠️ Alignment timeout - shooting anyway");
                    telemetry.update();
                }

                // Brief stabilization
                sleep(100);

            } catch (Exception e) {
                telemetry.addData("Alignment Error", e.getMessage());
                telemetry.update();
            }
        } else {
            telemetry.addLine("⚠️ No Limelight - shooting without alignment");
            telemetry.update();
        }

        // STEP 3: Execute shot
        telemetry.addLine("🔥 FIRING!");
        telemetry.addData("RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
        telemetry.addData("Ramp Angle", String.format(Locale.US, "%.1f°", rampController.getAngle()));
        telemetry.update();

        // Run intake and transfer to shoot
        transferController.transferFull();
        intakeController.intakeFull();

        // Continue updating shooter PID during shot
        ElapsedTime shotTimer = new ElapsedTime();
        while (opModeIsActive() && shotTimer.milliseconds() < SHOOT_DURATION) {
            shooterController.updatePID();
            sleep(50);
        }

        // Stop intake and transfer (but NOT the shooter!)
        transferController.transferStop();
        intakeController.intakeStop();

        telemetry.clear();
        telemetry.addLine("=== ✅ SHOT COMPLETE ===");
        telemetry.addData("Shooter Still Running", "YES ✅");
        telemetry.addData("RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
        telemetry.update();
    }

    /**
     * Move forward for intake while maintaining shooter RPM
     */
    @SuppressLint("DefaultLocale")
    private void ForwardForIntakeWithShooterUpdate(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX + distanceInches;

        telemetry.addData("Starting X", String.format(Locale.US, "%.2f in", startX));
        telemetry.addData("Target X", String.format(Locale.US, "%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move forward at intake speed
        driveController.tankDriveVelocityNormalized(INTAKE_SPEED, INTAKE_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            // IMPORTANT: Update shooter PID to maintain RPM
            shooterController.updatePID();

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(currentX - startX);

            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format(Locale.US, "%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", distanceInches));
            telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
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
     * Move backward while maintaining shooter RPM
     */
    @SuppressLint("DefaultLocale")
    private void moveBackwardWithOdometryAndShooterUpdate(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX - distanceInches;

        telemetry.addData("Starting X", String.format(Locale.US, "%.2f in", startX));
        telemetry.addData("Target X", String.format(Locale.US, "%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move backward at constant speed
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            // IMPORTANT: Update shooter PID to maintain RPM
            shooterController.updatePID();

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(startX - currentX);

            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format(Locale.US, "%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", distanceInches));
            telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
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
     * Move backward (initial movement, shooter is just starting up)
     */
    @SuppressLint("DefaultLocale")
    private void moveBackwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX - distanceInches;

        telemetry.addData("Starting X", String.format(Locale.US, "%.2f in", startX));
        telemetry.addData("Target X", String.format(Locale.US, "%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move backward at constant speed
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            // Update shooter PID even during initial movement
            shooterController.updatePID();

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(startX - currentX);

            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", currentX));
            telemetry.addData("Distance Moved", String.format(Locale.US, "%.2f in", distanceMoved));
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", distanceInches));
            telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
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

    @SuppressLint("DefaultLocale")
    private void turnToHeading(double targetHeadingDeg) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        final double HEADING_THRESHOLD = Math.toRadians(2.0);
        final double TURN_POWER = 0.3;

        while (opModeIsActive()) {
            // Keep updating shooter PID during turns
            shooterController.updatePID();

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
            telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f", shooterController.getShooterRPM()));
            telemetry.update();

            sleep(20);
        }
    }
}