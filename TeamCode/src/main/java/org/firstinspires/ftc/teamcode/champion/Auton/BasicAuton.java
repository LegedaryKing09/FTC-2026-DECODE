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

        // Step 5: New Movement Sequence
        telemetry.clear();
        telemetry.addLine("=== STEP 5: MOVEMENT SEQUENCE ===");
        telemetry.update();

        // Step 5.1: Turn left 90 degrees
        turnLeft90Degrees();

        // Step 5.2: Move forward 10 inches with intake on
        telemetry.clear();
        telemetry.addLine("=== STEP 5.2: FORWARD MOVEMENT WITH INTAKE ===");
        telemetry.addData("Target Distance", "10.0 inches");
        telemetry.update();

        intakeController.intakeFull(); // Turn on intake
        driveForwardWithOdometry(10.0);

        telemetry.addLine("✓ Forward movement with intake complete");
        telemetry.update();
        sleep(500);

        // Step 5.3: Move backward 10 inches (intake remains on)
        telemetry.clear();
        telemetry.addLine("=== STEP 5.3: BACKWARD MOVEMENT ===");
        telemetry.addData("Target Distance", "10.0 inches");
        telemetry.update();

        driveBackwardWithOdometry(10.0);

        telemetry.addLine("✓ Backward movement complete");
        telemetry.update();
        sleep(500);

        // Step 5.4: Turn right 90 degrees
        turnRight90Degrees();

        // Step 5.5: Activate shooting mechanism
        telemetry.clear();
        telemetry.addLine("=== STEP 5.5: ACTIVATING SHOOTER ===");
        telemetry.update();

        // Turn off intake before shooting
        intakeController.intakeStop();

        // Activate shooting mechanism (reuse existing shooting logic)
        transferController.transferFull();
        shooterController.updatePID();

        telemetry.addLine("✓ Shooting mechanism activated");
        telemetry.update();
        sleep(1000); // Brief activation time

        // Turn off transfer after shooting
        transferController.transferStop();

        telemetry.addLine("✓ Movement sequence complete");
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
     * Drive backward a specified distance using odometry with IMU-based heading correction
     */
    private void driveBackwardWithOdometry(double distance) throws InterruptedException {
        telemetry.addLine("=== DRIVING BACKWARD WITH ODOMETRY ===");
        telemetry.addData("Target Distance", "%.1f inches", distance);
        telemetry.update();

        // Update odometry to get fresh position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startY = driveController.getY();
        double startHeading = driveController.getHeading();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 8000; // 8 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Calculate distance traveled using position delta
            double currentX = driveController.getX();
            double currentY = driveController.getY();
            double deltaX = startX - currentX; // Negative X movement for backward
            double deltaY = startY - currentY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Check if we've reached the target distance
            if (distanceTraveled >= distance) {
                break;
            }

            // Calculate heading correction for straight movement
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);

            // Adjust motor speeds for straight backward movement with heading correction
            double baseSpeed = -DRIVE_SPEED; // Negative for backward
            double correction = headingError * 0.02; // Proportional correction
            double leftSpeed = baseSpeed - correction;
            double rightSpeed = baseSpeed + correction;

            driveController.tankDrive(leftSpeed, rightSpeed);

            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
            telemetry.addData("Current Heading", "%.2f°", currentHeading);
            telemetry.addData("Heading Error", "%.2f°", headingError);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
        telemetry.addLine("✓ Backward movement complete");
        telemetry.update();
        sleep(500);
    }

     /**
      * Drive forward a specified distance using odometry with IMU-based heading correction
      */
     private void driveForwardWithOdometry(double distance) throws InterruptedException {
         telemetry.addLine("=== DRIVING FORWARD WITH ODOMETRY ===");
         telemetry.addData("Target Distance", "%.1f inches", distance);
         telemetry.update();

         // Update odometry to get fresh position
         driveController.updateOdometry();
         double startX = driveController.getX();
         double startY = driveController.getY();
         double startHeading = driveController.getHeading();

         long driveStartTime = System.currentTimeMillis();
         long maxDriveTimeMs = 8000; // 8 second timeout

         while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
             driveController.updateOdometry();

             // Calculate distance traveled using position delta
             double currentX = driveController.getX();
             double currentY = driveController.getY();
             double deltaX = currentX - startX; // Positive X movement for forward
             double deltaY = currentY - startY;
             double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

             // Check if we've reached the target distance
             if (distanceTraveled >= distance) {
                 break;
             }

             // Calculate heading correction for straight movement
             double currentHeading = driveController.getHeading();
             double headingError = normalizeAngle(startHeading - currentHeading);

             // Adjust motor speeds for straight forward movement with heading correction
             double baseSpeed = DRIVE_SPEED; // Positive for forward
             double correction = headingError * 0.02; // Proportional correction
             double leftSpeed = baseSpeed + correction;
             double rightSpeed = baseSpeed - correction;

             driveController.tankDrive(leftSpeed, rightSpeed);

             telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
             telemetry.addData("Target Distance", "%.1f inches", distance);
             telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
             telemetry.addData("Current Heading", "%.2f°", currentHeading);
             telemetry.addData("Heading Error", "%.2f°", headingError);
             telemetry.update();

             sleep(20);
         }

         driveController.stopDrive();
         telemetry.addLine("✓ Forward movement complete");
         telemetry.update();
         sleep(500);
     }

     /**
      * Turn left 90 degrees using IMU-based closed-loop control
      */
     private void turnLeft90Degrees() throws InterruptedException {
         telemetry.addLine("=== TURNING LEFT 90 DEGREES (IMU-BASED) ===");
         telemetry.update();

         // Update odometry to get fresh heading
         driveController.updateOdometry();
         double startHeading = driveController.getHeadingDegrees();
         double targetHeading = startHeading - 90.0;

         // Normalize target heading to -180 to 180 range
         targetHeading = normalizeAngle(targetHeading);

         telemetry.addData("Start Heading", "%.2f°", startHeading);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.update();

         double turnSpeed = 0.3; // Reduced for better control
         double minTurnSpeed = 0.15; // Minimum speed to maintain turning
         double headingTolerance = 2.0; // Degrees tolerance
         double slowdownAngle = 15.0; // Start slowing down within this many degrees

         long timeoutMs = 3000; // 3 second timeout
         long turnStartTime = System.currentTimeMillis();

         while (opModeIsActive() && (System.currentTimeMillis() - turnStartTime) < timeoutMs) {
             // Update odometry to get current heading
             driveController.updateOdometry();
             double currentHeading = driveController.getHeadingDegrees();

             // Calculate shortest angle to target
             double headingError = getAngleError(targetHeading, currentHeading);

             // Check if we're within tolerance
             if (Math.abs(headingError) <= headingTolerance) {
                 break;
             }

             // Dynamic speed based on error
             double speed = turnSpeed;
             if (Math.abs(headingError) <= slowdownAngle) {
                 // Slow down as we approach target
                 speed = minTurnSpeed + (turnSpeed - minTurnSpeed) * (Math.abs(headingError) / slowdownAngle);
             }

             // Turn left (counter-clockwise): left side reverse, right side forward
             driveController.tankDrive(-speed, speed);

             telemetry.addData("Current Heading", "%.2f°", currentHeading);
             telemetry.addData("Heading Error", "%.2f°", headingError);
             telemetry.addData("Turn Speed", "%.3f", speed);
             telemetry.addData("Target Heading", "%.2f°", targetHeading);
             telemetry.update();

             sleep(20);
         }

         driveController.stopDrive();
         driveController.updateOdometry();
         double finalHeading = driveController.getHeadingDegrees();
         double finalError = getAngleError(targetHeading, finalHeading);

         telemetry.addLine("✓ Left turn complete");
         telemetry.addData("Final Heading", "%.2f°", finalHeading);
         telemetry.addData("Final Error", "%.2f°", finalError);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.update();
         sleep(500);
     }

     /**
      * Normalize angle to -180 to 180 degrees
      */
     private double normalizeAngle(double angle) {
         while (angle > 180) angle -= 360;
         while (angle <= -180) angle += 360;
         return angle;
     }

     /**
      * Calculate the shortest angle error between target and current heading
      */
     private double getAngleError(double target, double current) {
         double error = target - current;
         return normalizeAngle(error);
     }

     /**
      * Turn right 90 degrees using IMU-based closed-loop control
      */
     private void turnRight90Degrees() throws InterruptedException {
         telemetry.addLine("=== TURNING RIGHT 90 DEGREES (IMU-BASED) ===");
         telemetry.update();

         // Update odometry to get fresh heading
         driveController.updateOdometry();
         double startHeading = driveController.getHeadingDegrees();
         double targetHeading = startHeading + 90.0;

         // Normalize target heading to -180 to 180 range
         targetHeading = normalizeAngle(targetHeading);

         telemetry.addData("Start Heading", "%.2f°", startHeading);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.update();

         double turnSpeed = 0.3; // Reduced for better control
         double minTurnSpeed = 0.15; // Minimum speed to maintain turning
         double headingTolerance = 2.0; // Degrees tolerance
         double slowdownAngle = 15.0; // Start slowing down within this many degrees

         long timeoutMs = 3000; // 3 second timeout
         long turnStartTime = System.currentTimeMillis();

         while (opModeIsActive() && (System.currentTimeMillis() - turnStartTime) < timeoutMs) {
             // Update odometry to get current heading
             driveController.updateOdometry();
             double currentHeading = driveController.getHeadingDegrees();

             // Calculate shortest angle to target
             double headingError = getAngleError(targetHeading, currentHeading);

             // Check if we're within tolerance
             if (Math.abs(headingError) <= headingTolerance) {
                 break;
             }

             // Dynamic speed based on error
             double speed = turnSpeed;
             if (Math.abs(headingError) <= slowdownAngle) {
                 // Slow down as we approach target
                 speed = minTurnSpeed + (turnSpeed - minTurnSpeed) * (Math.abs(headingError) / slowdownAngle);
             }

             // Turn right (clockwise): left side forward, right side reverse
             driveController.tankDrive(speed, -speed);

             telemetry.addData("Current Heading", "%.2f°", currentHeading);
             telemetry.addData("Heading Error", "%.2f°", headingError);
             telemetry.addData("Turn Speed", "%.3f", speed);
             telemetry.addData("Target Heading", "%.2f°", targetHeading);
             telemetry.update();

             sleep(20);
         }

         driveController.stopDrive();
         driveController.updateOdometry();
         double finalHeading = driveController.getHeadingDegrees();
         double finalError = getAngleError(targetHeading, finalHeading);

         telemetry.addLine("✓ Right turn complete");
         telemetry.addData("Final Heading", "%.2f°", finalHeading);
         telemetry.addData("Final Error", "%.2f°", finalError);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.update();
         sleep(500);
     }
}