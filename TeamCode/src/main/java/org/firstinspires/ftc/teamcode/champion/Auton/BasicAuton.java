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

    // Autonomous parameters
    public static double SHOOTER_START_RPM = 2750.0;
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double MOVEMENT_SPEED = 0.3; // Conservative speed for odometry

    public static double INTAKE_SPEED = 0.15; // Conservative speed for odometry
    public static long AUTO_SHOOT_TIMEOUT = 15000; // Maximum time to wait for auto-shoot sequence
    public static long SETTLE_TIME = 500; // Time to settle after movement before shooting

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

        // 1. Start shooter spinning early
        telemetry.addLine("Starting shooter...");
        telemetry.update();
        shooterController.setShooterRPM(SHOOTER_START_RPM);

        // 2. Execute backward movement
        telemetry.addLine("Moving backward...");
        telemetry.update();
        moveBackwardWithOdometry(30.0);

        // 3. Settle after movement
        telemetry.addLine("Settling...");
        telemetry.update();
        sleep(SETTLE_TIME);

        // 4. Wait for shooter to reach target RPM before shooting
        telemetry.addLine("Waiting for shooter to spin up...");
        telemetry.update();
        ElapsedTime spinUpTimer1 = new ElapsedTime();
        while (opModeIsActive() && spinUpTimer1.seconds() < 5.0) {
            shooterController.updatePID();

            telemetry.addData("Current RPM", String.format("%.0f", shooterController.getShooterRPM()));
            telemetry.addData("Target RPM", String.format("%.0f", shooterController.getTargetRPM()));
            telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✅ YES" : "⏳ Spinning up...");
            telemetry.update();

            if (shooterController.isAtTargetRPM()) {
                telemetry.addLine("✅ Shooter ready!");
                telemetry.update();
                sleep(500); // Extra settling time
                break;
            }
            sleep(50);
        }

        // Execute auto-shoot sequence with longer timeout for 2 balls
        telemetry.addLine("Starting auto-shoot sequence...");
        telemetry.update();
        executeAutoShootSequence(); // 15 seconds for 2 balls

        // Settle after movement
        telemetry.addLine("Settling...");
        telemetry.update();
        sleep(SETTLE_TIME);

        // Turn left 45 degrees
        telemetry.addLine("Turning left 45 degrees...");
        telemetry.update();
        turnToHeading(39.0); // Positive angle = counterclockwise (left)
        sleep(300); // Small settle time after turn

        // Move forward to pick up 2 balls
        telemetry.addLine("Moving forward 21.5 inches...");
        telemetry.update();
        intakeController.intakeFull(); // Start intake while moving
        moveForwardForIntake(21.5);
        sleep(SETTLE_TIME); // Settle after movement


        sleep(1000);
        intakeController.intakeStop();

        // Execute backward movement
        telemetry.addLine("Moving backward...");
        telemetry.update();
        moveBackwardWithOdometry(10.0);

        // Turn right 50 degrees
        telemetry.addLine("=== STARTING NEW SEQUENCE ===");
        telemetry.addLine("Turning left 50 degrees...");
        telemetry.update();
        turnToHeading(14.0); // Positive angle = counterclockwise (left)
        sleep(300); // Small settle time after turn

        telemetry.addLine("Starting shooter...");
        telemetry.update();
        shooterController.setShooterRPM(SHOOTER_START_RPM);

        // Settle after movement
        telemetry.addLine("Settling...");
        telemetry.update();
        sleep(SETTLE_TIME);

        // Wait for shooter to reach target RPM before shooting
        telemetry.addLine("Waiting for shooter to spin up...");
        telemetry.update();
        ElapsedTime spinUpTimer2 = new ElapsedTime();
        while (opModeIsActive() && spinUpTimer2.seconds() < 5.0) {
            shooterController.updatePID();

            telemetry.addData("Current RPM", String.format("%.0f", shooterController.getShooterRPM()));
            telemetry.addData("Target RPM", String.format("%.0f", shooterController.getTargetRPM()));
            telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✅ YES" : "⏳ Spinning up...");
            telemetry.update();

            if (shooterController.isAtTargetRPM()) {
                telemetry.addLine("✅ Shooter ready!");
                telemetry.update();
                sleep(500); // Extra settling time
                break;
            }
            sleep(50);
        }

        // Execute auto-shoot sequence with longer timeout for 2 balls
        telemetry.addLine("Starting auto-shoot sequence...");
        telemetry.update();
        executeAutoShootSequence(); // 15 seconds for 2 balls


    }
    /**
     * Move forward a specified distance using odometry
     * Add this method to your BasicAuton class
     */
    @SuppressLint("DefaultLocale")
    private void moveForwardWithOdometry(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX + distanceInches; // Positive X is forward

        telemetry.addData("Starting X", String.format("%.2f in", startX));
        telemetry.addData("Target X", String.format("%.2f in", targetX));
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

        telemetry.addLine("✅ Forward movement complete");
        telemetry.update();
        sleep(200);
    }

    @SuppressLint("DefaultLocale")
    private void moveForwardForIntake(double distanceInches) {
        // Record starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetX = startX + distanceInches; // Positive X is forward

        telemetry.addData("Starting X", String.format("%.2f in", startX));
        telemetry.addData("Target X", String.format("%.2f in", targetX));
        telemetry.update();

        // Set drive mode to velocity for precise control
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        // Move forward at constant speed
        driveController.tankDriveVelocityNormalized(INTAKE_SPEED, INTAKE_SPEED);

        // Monitor progress
        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(currentX - startX);

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

        telemetry.addLine("✅ Forward intake movement complete");
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
    @SuppressLint("DefaultLocale")
    private void executeAutoShootSequence() {
        // Trigger the auto-shoot sequence (runs in separate thread)
        autoShootController.executeDistanceBasedAutoShoot();

        transferController.transferFull();
        intakeController.intakeFull();

        // Wait for auto-shoot to complete or timeout
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && autoShootController.isAutoShooting() &&
                timer.milliseconds() < AUTO_SHOOT_TIMEOUT) {

            // Update shooter PID during the sequence, keep the transfer and intake running continuously
            shooterController.updatePID();
            transferController.transferFull();
            intakeController.intakeFull();

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

        // Keep transfer running for extra time after auto-shoot thinks it's done
        ElapsedTime extraTime = new ElapsedTime();
        while (opModeIsActive() && extraTime.seconds() < 4.0) {
            shooterController.updatePID(); // Keep shooter spinning
            transferController.transferFull(); // Keep transfer running
            intakeController.intakeFull(); // Keep intake running

            telemetry.addData("Extra Time", String.format("%.1f s", extraTime.seconds()));
            telemetry.addData("Current RPM", String.format("%.0f", shooterController.getShooterRPM()));
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

        sleep(500);
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

            telemetry.addData("Target", String.format("(%.1f, %.1f)", x, y));
            telemetry.addData("Current", String.format("(%.1f, %.1f)", currentPose.position.x, currentPose.position.y));
            telemetry.addData("Dist to Target", String.format("%.1f in", distToEnd));
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

            telemetry.addData("Target Heading", String.format("%.1f°", targetHeadingDeg));
            telemetry.addData("Current Heading", String.format("%.1f°", Math.toDegrees(currentHeading)));
            telemetry.update();

            sleep(20);
        }
    }
}

