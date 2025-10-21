package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.BallAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.PurePursuitController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;
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

    // Heading correction parameters for better turning accuracy
    public static double HEADING_CORRECTION_FACTOR = 0.03; // Increased for better correction
    public static double HEADING_TOLERANCE_DEGREES = 1.5; // Tighter tolerance for more precise turns
    public static double MINIMUM_TURN_SPEED = 0.12; // Minimum speed to maintain turning
    public static double HEADING_SMOOTHING_FACTOR = 0.3; // Smoothing for heading readings

    // Controllers
    private SixWheelDriveController driveController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private TransferController transferController;
    private LimelightAlignmentController limelightController;
    private BallAlignmentController ballAlignmentController;
    private RampController rampController;
    private PurePursuitController purePursuitController;
    private AutoShootController autoShootController;

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

            // Move ramp to 170 degrees at autonomous start
            if (rampController != null) {
                telemetry.clear();
                telemetry.addLine("=== MOVING RAMP TO 170 DEGREES ===");
                telemetry.update();

                rampController.setAngle(170.0);

                telemetry.addLine("✓ Ramp moved to 170 degrees");
                telemetry.addData("Ramp Angle", "%.1f°", rampController.getAngle());
                telemetry.update();
                sleep(1000); // Give ramp time to move
            }

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

        // Initialize drive controller first (required by other controllers)
        telemetry.addLine("Initializing drive controller...");
        telemetry.update();
        driveController = new SixWheelDriveController(this);
        telemetry.addLine("✓ Drive controller initialized");

        // Initialize shooter components
        telemetry.addLine("Initializing shooter components...");
        telemetry.update();
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        transferController = new TransferController(this);
        telemetry.addLine("✓ Shooter components initialized");

        // Initialize ramp controller
        try {
            telemetry.addLine("Initializing ramp controller...");
            telemetry.update();
            rampController = new RampController(this);
            rampController.setTo0Degrees(); // Initialize ramp at 0 degrees
            telemetry.addLine("✓ Ramp controller initialized at 0 degrees");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Ramp controller failed to init: " + e.getMessage());
            rampController = null;
        }

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

        // Initialize pure pursuit controller
        try {
            telemetry.addLine("Initializing pure pursuit controller...");
            telemetry.update();
            purePursuitController = new PurePursuitController();
            telemetry.addLine("✓ Pure pursuit controller initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Pure pursuit controller failed to init: " + e.getMessage());
            purePursuitController = null;
        }

        // Initialize auto shoot controller
        try {
            telemetry.addLine("Initializing auto shoot controller...");
            telemetry.update();
            autoShootController = new AutoShootController(this,
                    driveController,
                    shooterController,
                    intakeController,
                    transferController,
                    limelightController,
                    rampController);
            telemetry.addLine("✓ Auto shoot controller initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Auto shoot controller failed to init: " + e.getMessage());
            autoShootController = null;
        }

        telemetry.addLine("All controllers initialized successfully");
        telemetry.update();
        sleep(500); // Brief pause to ensure all hardware is ready
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

            // Update odometry more frequently for better accuracy
            driveController.updateOdometry();

            telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
            telemetry.addData("Position X", "%.2f in", driveController.getX());
            telemetry.addData("Position Y", "%.2f in", driveController.getY());
            telemetry.addData("Heading", "%.2f°", driveController.getHeadingDegrees());
            telemetry.update();

            sleep(10); // Faster update rate for better responsiveness
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

        // Step 3 & 4: Auto Shooting Sequence
        telemetry.clear();
        telemetry.addLine("=== STEPS 3 & 4: AUTO SHOOTING SEQUENCE ===");
        telemetry.update();

        if (autoShootController != null) {
            autoShootController.executeDistanceBasedAutoShoot();

            // Wait for auto-shoot to complete
            long autoShootStartTime = System.currentTimeMillis();
            while (opModeIsActive() &&
                   autoShootController.isAutoShooting() &&
                   (System.currentTimeMillis() - autoShootStartTime) < 15000) { // 15 second timeout

                driveController.updateOdometry();
                autoShootController.addTelemetry(telemetry);
                telemetry.update();

                sleep(20);
            }

            if (!autoShootController.isAutoShooting()) {
                telemetry.addLine("✓ Auto shooting sequence complete");
            } else {
                telemetry.addLine("⚠ Auto shooting sequence timeout");
            }
        } else {
            telemetry.addLine("⚠ Auto shoot controller not available");
        }
        telemetry.update();
        sleep(500);

        // Step 5: New Movement Sequence
        telemetry.clear();
        telemetry.addLine("=== STEP 5: PURE PURSUIT MOVEMENT SEQUENCE ===");
        telemetry.update();

        if (purePursuitController != null) {
            // Step 5.1: Reset odometry to (0,0) at the start of the new movement sequence
            driveController.resetOdometry();
            telemetry.addLine("✓ Odometry reset to (0,0)");
            telemetry.update();
            sleep(500);

            // Step 5.2: Use PurePursuitController to navigate to point (0, 10) with intake on
            telemetry.clear();
            telemetry.addLine("=== STEP 5.2: NAVIGATING TO (0, 10) WITH INTAKE ===");
            telemetry.addData("Target Point", "(0, 10)");
            telemetry.update();

            intakeController.intakeFull(); // Turn on intake
            purePursuitController.setTargetPosition(new Vector2d(0, -10));

            long navigationStartTime = System.currentTimeMillis();
            long maxNavigationTime = 10000; // 10 second timeout

            while (opModeIsActive() && (System.currentTimeMillis() - navigationStartTime) < maxNavigationTime) {
                driveController.updateOdometry();

                Pose2d currentPose = new Pose2d(
                    new Vector2d(driveController.getX(), driveController.getY()),
                    driveController.getHeading()
                );

                double[] powers = purePursuitController.update(currentPose);
                driveController.tankDriveVelocityNormalized(powers[0], powers[1]);

                // Check if we've reached the target (within 1 inch)
                double distanceToTarget = Math.sqrt(
                    Math.pow(driveController.getX() - 0, 2) +
                    Math.pow(driveController.getY() - 10, 2)
                );

                telemetry.addData("Current Position", "(%.1f, %.1f)", driveController.getX(), driveController.getY());
                telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
                telemetry.addData("Intake", "ACTIVE");
                telemetry.update();

                if (distanceToTarget < 1.0) {
                    break;
                }

                sleep(20);
            }

            driveController.stopDrive();
            telemetry.addLine("✓ Navigation to (0, 10) complete");
            telemetry.update();
            sleep(500);

            // Step 5.3: Reset odometry to (0,0) before the second navigation
            driveController.resetOdometry();
            telemetry.addLine("✓ Odometry reset to (0,0) before second navigation");
            telemetry.update();
            sleep(500);

            // Step 5.4: Navigate to point (-10, 0) going backwards (to avoid 180 degree turn requirement)
            telemetry.clear();
            telemetry.addLine("=== STEP 5.4: NAVIGATING TO (-10, 0) BACKWARDS ===");
            telemetry.addData("Target Point", "(-10, 0)");
            telemetry.addData("Direction", "BACKWARDS");
            telemetry.update();

            purePursuitController.setTargetPosition(new Vector2d(-10, 0));

            navigationStartTime = System.currentTimeMillis();

            while (opModeIsActive() && (System.currentTimeMillis() - navigationStartTime) < maxNavigationTime) {
                driveController.updateOdometry();

                Pose2d currentPose = new Pose2d(
                    new Vector2d(driveController.getX(), driveController.getY()),
                    driveController.getHeading()
                );

                double[] powers = purePursuitController.update(currentPose);
                // Reverse the powers for backward movement
                driveController.tankDriveVelocityNormalized(-powers[0], -powers[1]);

                // Check if we've reached the target (within 1 inch)
                double distanceToTarget = Math.sqrt(
                    Math.pow(driveController.getX() - (-10), 2) +
                    Math.pow(driveController.getY() - 0, 2)
                );

                telemetry.addData("Current Position", "(%.1f, %.1f)", driveController.getX(), driveController.getY());
                telemetry.addData("Distance to Target", "%.1f inches", distanceToTarget);
                telemetry.addData("Direction", "BACKWARDS");
                telemetry.update();

                if (distanceToTarget < 1.0) {
                    break;
                }

                sleep(20);
            }

            driveController.stopDrive();
            telemetry.addLine("✓ Backward navigation to (-10, 0) complete");
            telemetry.update();
            sleep(500);

        } else {
            telemetry.addLine("⚠ Pure pursuit controller not available - skipping movement sequence");
        }

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

            // Adjust motor speeds for straight backward movement with improved heading correction
            double baseSpeed = -DRIVE_SPEED; // Negative for backward
            double correction = headingError * HEADING_CORRECTION_FACTOR; // Use configurable correction factor
            double leftSpeed = baseSpeed - correction;
            double rightSpeed = baseSpeed + correction;

            // Apply speed limits to prevent excessive correction
            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

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

             // Adjust motor speeds for straight forward movement with improved heading correction
             double baseSpeed = DRIVE_SPEED; // Positive for forward
             double correction = headingError * HEADING_CORRECTION_FACTOR; // Use configurable correction factor
             double leftSpeed = baseSpeed + correction;
             double rightSpeed = baseSpeed - correction;

             // Apply speed limits to prevent excessive correction
             leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
             rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

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
         telemetry.addData("Raw IMU Heading", "%.2f°", Math.toDegrees(driveController.getHeading()));
         telemetry.update();

         double turnSpeed = 0.3; // Reduced for better control
         double minTurnSpeed = MINIMUM_TURN_SPEED; // Use configurable minimum speed
         double headingTolerance = HEADING_TOLERANCE_DEGREES; // Use configurable tolerance
         double slowdownAngle = 15.0; // Start slowing down within this many degrees

         long timeoutMs = 3000; // 3 second timeout
         long turnStartTime = System.currentTimeMillis();

         int loopCount = 0;
         double initialRawHeading = driveController.getHeading(); // Raw radians

         while (opModeIsActive() && (System.currentTimeMillis() - turnStartTime) < timeoutMs) {
             loopCount++;
             // Update odometry to get current heading
             driveController.updateOdometry();
             double currentHeading = driveController.getHeadingDegrees();
             double currentRawHeading = driveController.getHeading(); // Raw radians

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

             // Debug logging every 5 loops to avoid telemetry spam
             if (loopCount % 5 == 0) {
                 telemetry.addData("Loop", loopCount);
                 telemetry.addData("Current Heading", "%.2f°", currentHeading);
                 telemetry.addData("Raw IMU (rad)", "%.4f", currentRawHeading);
                 telemetry.addData("Heading Delta", "%.2f°", currentHeading - startHeading);
                 telemetry.addData("Heading Error", "%.2f°", headingError);
                 telemetry.addData("Turn Speed", "%.3f", speed);
                 telemetry.addData("Target Heading", "%.2f°", targetHeading);
                 telemetry.addData("Time Elapsed", "%.1fs", (System.currentTimeMillis() - turnStartTime) / 1000.0);
                 telemetry.update();
             }

             sleep(20);
         }

         driveController.stopDrive();
         driveController.updateOdometry();
         double finalHeading = driveController.getHeadingDegrees();
         double finalRawHeading = driveController.getHeading(); // Raw radians
         double finalError = getAngleError(targetHeading, finalHeading);
         double totalTurnDegrees = finalHeading - startHeading;
         double rawTurnRadians = finalRawHeading - initialRawHeading;
         double rawTurnDegrees = Math.toDegrees(rawTurnRadians);

         telemetry.addLine("✓ Left turn complete");
         telemetry.addData("Final Heading", "%.2f°", finalHeading);
         telemetry.addData("Final Raw IMU (rad)", "%.4f", finalRawHeading);
         telemetry.addData("Total Turn (processed)", "%.2f°", totalTurnDegrees);
         telemetry.addData("Total Turn (raw IMU)", "%.2f°", rawTurnDegrees);
         telemetry.addData("Ratio (raw/processed)", "%.2f", Math.abs(rawTurnDegrees / totalTurnDegrees));
         telemetry.addData("Final Error", "%.2f°", finalError);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.addData("Loops Executed", loopCount);
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
         telemetry.addData("Raw IMU Heading", "%.2f°", Math.toDegrees(driveController.getHeading()));
         telemetry.update();

         double turnSpeed = 0.3; // Reduced for better control
         double minTurnSpeed = MINIMUM_TURN_SPEED; // Use configurable minimum speed
         double headingTolerance = HEADING_TOLERANCE_DEGREES; // Use configurable tolerance
         double slowdownAngle = 15.0; // Start slowing down within this many degrees

         long timeoutMs = 3000; // 3 second timeout
         long turnStartTime = System.currentTimeMillis();

         int loopCount = 0;
         double initialRawHeading = driveController.getHeading(); // Raw radians

         while (opModeIsActive() && (System.currentTimeMillis() - turnStartTime) < timeoutMs) {
             loopCount++;
             // Update odometry to get current heading
             driveController.updateOdometry();
             double currentHeading = driveController.getHeadingDegrees();
             double currentRawHeading = driveController.getHeading(); // Raw radians

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

             // Debug logging every 5 loops to avoid telemetry spam
             if (loopCount % 5 == 0) {
                 telemetry.addData("Loop", loopCount);
                 telemetry.addData("Current Heading", "%.2f°", currentHeading);
                 telemetry.addData("Raw IMU (rad)", "%.4f", currentRawHeading);
                 telemetry.addData("Heading Delta", "%.2f°", currentHeading - startHeading);
                 telemetry.addData("Heading Error", "%.2f°", headingError);
                 telemetry.addData("Turn Speed", "%.3f", speed);
                 telemetry.addData("Target Heading", "%.2f°", targetHeading);
                 telemetry.addData("Time Elapsed", "%.1fs", (System.currentTimeMillis() - turnStartTime) / 1000.0);
                 telemetry.update();
             }

             sleep(20);
         }

         driveController.stopDrive();
         driveController.updateOdometry();
         double finalHeading = driveController.getHeadingDegrees();
         double finalRawHeading = driveController.getHeading(); // Raw radians
         double finalError = getAngleError(targetHeading, finalHeading);
         double totalTurnDegrees = finalHeading - startHeading;
         double rawTurnRadians = finalRawHeading - initialRawHeading;
         double rawTurnDegrees = Math.toDegrees(rawTurnRadians);

         telemetry.addLine("✓ Right turn complete");
         telemetry.addData("Final Heading", "%.2f°", finalHeading);
         telemetry.addData("Final Raw IMU (rad)", "%.4f", finalRawHeading);
         telemetry.addData("Total Turn (processed)", "%.2f°", totalTurnDegrees);
         telemetry.addData("Total Turn (raw IMU)", "%.2f°", rawTurnDegrees);
         telemetry.addData("Ratio (raw/processed)", "%.2f", Math.abs(rawTurnDegrees / totalTurnDegrees));
         telemetry.addData("Final Error", "%.2f°", finalError);
         telemetry.addData("Target Heading", "%.2f°", targetHeading);
         telemetry.addData("Loops Executed", loopCount);
         telemetry.update();
         sleep(500);
     }
}