package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.BallAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;

@Config
@Autonomous(name = "Basic Autonomous", group = "Competition")
public class BasicAuton extends LinearOpMode {

    // Configurable parameters
    public static double SHOOTER_TARGET_RPM = 2750.0;
    public static double BACKWARD_DISTANCE_INCHES = 48.0;
    public static double DRIVE_SPEED = 0.5;
    public static double BALL_PURSUIT_OVERSHOOT_INCHES = 6.0;
    public static long SHOOTER_STARTUP_TIMEOUT_MS = 5000;
    public static long SHOOTING_DURATION_MS = 2000;
    public static long BALL_DETECTION_TIMEOUT_MS = 8000;

    // Controllers
    private SixWheelDriveController driveController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private TransferController transferController;
    private LimelightAlignmentController limelightController;
    private BallAlignmentController ballAlignmentController;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialize all controllers
            initializeControllers();

            // Display initialization status
            telemetry.addLine("=== SECOND AUTONOMOUS INITIALIZED ===");
            telemetry.addData("Shooter RPM", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("Backward Distance", "%.1f inches", BACKWARD_DISTANCE_INCHES);
            telemetry.addData("Ball Overshoot", "%.1f inches", BALL_PURSUIT_OVERSHOOT_INCHES);
            telemetry.addLine("Press START to begin");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            runtime.reset();

            // Execute autonomous sequence
            telemetry.addLine("Starting autonomous sequence...");
            telemetry.update();
            executeAutonomousSequence();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.update();

            // Log error but don't crash
            sleep(3000);
        }
    }

    private void initializeControllers() throws Exception {
        telemetry.addLine("=== INITIALIZING CONTROLLERS ===");
        telemetry.update();

        // Initialize drive controller
        telemetry.addLine("Initializing drive controller...");
        telemetry.update();
        driveController = new SixWheelDriveController(this);

        // Initialize shooter components
        telemetry.addLine("Initializing shooter components...");
        telemetry.update();
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        transferController = new TransferController(this);

        // Initialize Limelight alignment
        try {
            telemetry.addLine("Initializing Limelight alignment controller...");
            telemetry.update();
            limelightController = new LimelightAlignmentController(this, driveController);
            telemetry.addLine("✓ Limelight alignment controller initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Limelight alignment failed to init: " + e.getMessage());
            limelightController = null;
        }

        // Initialize ball alignment controller
        try {
            telemetry.addLine("Initializing ball alignment controller...");
            telemetry.update();
            ballAlignmentController = new BallAlignmentController(this);
            telemetry.addLine("✓ Ball alignment controller initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Ball alignment failed to init: " + e.getMessage());
            ballAlignmentController = null;
        }

        telemetry.addLine("All controllers initialized successfully");
        telemetry.update();
    }

    private void executeAutonomousSequence() throws InterruptedException {
        // Step 1: Shooter Startup
        telemetry.clear();
        telemetry.addLine("=== STEP 1: SHOOTER STARTUP ===");
        telemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.update();

        shooterController.setShooterRPM(SHOOTER_TARGET_RPM);

        // Wait for shooter to reach RPM
        long shooterStartTime = System.currentTimeMillis();
        while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                (System.currentTimeMillis() - shooterStartTime) < SHOOTER_STARTUP_TIMEOUT_MS) {
            shooterController.updatePID();
            driveController.updateOdometry();

            telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
            telemetry.update();

            sleep(20);
        }

        if (shooterController.isAtTargetRPM()) {
            telemetry.addLine("✓ Shooter at target RPM");
        } else {
            telemetry.addLine("⚠ Shooter startup timeout - continuing anyway");
        }
        telemetry.update();
        sleep(500);

        // Step 2: Backward Movement
        telemetry.clear();
        telemetry.addLine("=== STEP 2: MOVING BACKWARD ===");
        telemetry.addData("Target Distance", "%.1f inches", BACKWARD_DISTANCE_INCHES);
        telemetry.update();

        driveBackwardWithOdometry(BACKWARD_DISTANCE_INCHES);

        telemetry.addLine("✓ Backward movement complete");
        telemetry.update();
        sleep(500);

        // Step 3: AprilTag Alignment
        telemetry.clear();
        telemetry.addLine("=== STEP 3: APRILTAG ALIGNMENT ===");
        telemetry.update();

        if (limelightController != null) {
            limelightController.startAlignment();

            // Wait for alignment to complete
            long alignmentStartTime = System.currentTimeMillis();
            while (opModeIsActive() &&
                   !limelightController.isAligned() &&
                   (System.currentTimeMillis() - alignmentStartTime) < 5000) {

                driveController.updateOdometry();
                limelightController.displayTelemetry();

                if (limelightController.hasTarget()) {
                    telemetry.addData("Target Error", "%.2f°", limelightController.getTargetError());
                }

                telemetry.update();
                sleep(20);
            }

            if (limelightController.isAligned()) {
                telemetry.addLine("✓ AprilTag alignment complete");
            } else {
                telemetry.addLine("⚠ AprilTag alignment timeout");
            }
        } else {
            telemetry.addLine("⚠ Limelight controller not available");
        }
        telemetry.update();
        sleep(500);

        // Step 4: Shooting Sequence
        telemetry.clear();
        telemetry.addLine("=== STEP 4: SHOOTING SEQUENCE ===");
        telemetry.update();

        // Turn on transfer mechanism
        transferController.transferFull();

        // Turn on intake to feed balls
        intakeController.intakeFull();

        // Wait for shooting duration
        long shootStartTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - shootStartTime) < SHOOTING_DURATION_MS) {
            shooterController.updatePID();
            driveController.updateOdometry();

            telemetry.addData("Shooting Progress", "%.1f seconds",
                (System.currentTimeMillis() - shootStartTime) / 1000.0);
            telemetry.addData("Shooter RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Transfer", "ACTIVE");
            telemetry.addData("Intake", "ACTIVE");
            telemetry.update();

            sleep(20);
        }

        // Turn off transfer
        transferController.transferStop();

        telemetry.addLine("✓ Shooting sequence complete");
        telemetry.update();
        sleep(500);

        // Step 5: Ball Detection
        telemetry.clear();
        telemetry.addLine("=== STEP 5: BALL DETECTION ===");
        telemetry.update();

        if (ballAlignmentController != null) {
            telemetry.addLine("Starting ball detection...");
            telemetry.update();

            ballAlignmentController.startTracking();

            long detectionStartTime = System.currentTimeMillis();
            boolean ballFound = false;

            while (opModeIsActive() &&
                   (System.currentTimeMillis() - detectionStartTime) < BALL_DETECTION_TIMEOUT_MS) {

                driveController.updateOdometry();
                ballAlignmentController.align();

                if (ballAlignmentController.hasBall()) {
                    ballFound = true;
                    telemetry.addLine("✓ Ball detected!");
                    telemetry.addData("Ball Distance", "%.1f in", ballAlignmentController.getBallDistance());
                    telemetry.addData("Ball Color", ballAlignmentController.getBallColor() == 1 ? "GREEN" : "PURPLE");
                    telemetry.update();
                    break;
                }

                ballAlignmentController.displayTelemetry();
                telemetry.update();
                sleep(20);
            }

            if (!ballFound) {
                telemetry.addLine("⚠ Ball detection timeout");
                telemetry.update();
                sleep(1000);
            }

            ballAlignmentController.stopTracking();
        } else {
            telemetry.addLine("⚠ Ball alignment controller not available");
            telemetry.update();
            sleep(2000);
        }

        // Step 6: Ball Pursuit
        telemetry.clear();
        telemetry.addLine("=== STEP 6: BALL PURSUIT ===");
        telemetry.update();

        if (ballAlignmentController != null && ballAlignmentController.hasBall()) {
            double ballDistance = ballAlignmentController.getBallDistance();

            if (ballDistance > 0) {
                // Calculate target distance (ball distance + overshoot)
                double targetDistance = ballDistance + BALL_PURSUIT_OVERSHOOT_INCHES;

                telemetry.addData("Ball Distance", "%.1f in", ballDistance);
                telemetry.addData("Overshoot", "%.1f in", BALL_PURSUIT_OVERSHOOT_INCHES);
                telemetry.addData("Target Distance", "%.1f in", targetDistance);
                telemetry.update();

                // Drive to ball with overshoot
                driveForwardWithOdometry(targetDistance);

                telemetry.addLine("✓ Ball pursuit complete");
            } else {
                telemetry.addLine("⚠ Invalid ball distance");
            }
        } else {
            telemetry.addLine("⚠ No ball detected for pursuit");
        }
        telemetry.update();
        sleep(500);

        // Step 7: Final Stop
        telemetry.clear();
        telemetry.addLine("=== STEP 7: FINAL STOP ===");
        telemetry.update();

        // Stop all systems
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        telemetry.addLine("✓ All systems stopped");
        telemetry.addData("Total Runtime", "%.2f seconds", runtime.seconds());
        telemetry.update();

        // Keep robot stopped
        while (opModeIsActive() && !isStopRequested()) {
            driveController.stopDrive();
            sleep(50);
        }
    }

    /**
     * Drive backward a specified distance using odometry only
     */
    private void driveBackwardWithOdometry(double distance) throws InterruptedException {
        double startX = driveController.getX();
        double startY = driveController.getY();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 8000; // 8 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Calculate distance traveled (backward movement - check X coordinate change)
            double currentX = driveController.getX();
            double currentY = driveController.getY();
            double distanceTraveled = Math.abs(startX - currentX);

            // Check if we've reached the target distance
            if (distanceTraveled >= distance) {
                break;
            }

            // Drive straight backward (negative X direction)
            driveController.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);

            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Drive forward a specified distance using odometry only
     */
    private void driveForwardWithOdometry(double distance) throws InterruptedException {
        double startX = driveController.getX();
        double startY = driveController.getY();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 8000; // 8 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Calculate distance traveled (forward movement - check X coordinate change)
            double currentX = driveController.getX();
            double currentY = driveController.getY();
            double distanceTraveled = Math.abs(currentX - startX);

            // Check if we've reached the target distance
            if (distanceTraveled >= distance) {
                break;
            }

            // Drive straight forward (positive X direction)
            driveController.tankDrive(DRIVE_SPEED, DRIVE_SPEED);

            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }
}