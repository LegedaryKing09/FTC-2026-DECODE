package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
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

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;

@Config
@Autonomous(name = "First Auton with Ball Collection", group = "Competition")
public class FirstAuton extends LinearOpMode {

    // Configurable parameters
    public static double DRIVE_DISTANCE_INCHES = 60.0;
    public static double TURN_DEGREES = 180.0;
    public static double DRIVE_SPEED = 0.6;
    public static double TURN_SPEED = 0.4;
    public static double POSITION_TOLERANCE_INCHES = 1.0;
    public static double HEADING_TOLERANCE_DEGREES = 2.0;
    public static long POST_TURN_DELAY_MS = 500;

    // Ball collection parameters
    public static double BALL_SEARCH_TURN_DEGREES = -90.0;  // Turn left
    public static double BALL_COLLECTION_DISTANCE = 36.0;   // Drive forward 36 inches
    public static double BALL_ALIGNMENT_TIMEOUT_MS = 8000;  // Max time to find and align with ball
    public static double BALL_COLLECTION_SPEED = 0.5;       // Speed while collecting ball

    // Controllers
    private SixWheelDriveController driveController;
    private PurePursuitController purePursuitController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private TransferController transferController;
    private RampController rampController;
    private LimelightAlignmentController limelightController;
    private AutoShootController autoShootController;
    private BallAlignmentController ballAlignmentController;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialize all controllers
            initializeControllers();

            // Display initialization status
            telemetry.addLine("=== SHOOTING AUTONOMOUS INITIALIZED ===");
            telemetry.addData("Drive Distance", "%.1f inches", DRIVE_DISTANCE_INCHES);
            telemetry.addData("Turn Angle", "%.1f degrees", TURN_DEGREES);
            telemetry.addData("Drive Speed", "%.1f", DRIVE_SPEED);
            telemetry.addData("Turn Speed", "%.1f", TURN_SPEED);
            telemetry.addLine("Press START to begin");
            telemetry.update();

            // Shooter will be started in the autonomous sequence - not here

            waitForStart();
            if (isStopRequested()) return;

            runtime.reset();

            // Set ramp to 30 degrees at start
            rampController.setAngle(30.0);

            // Execute autonomous sequence
            telemetry.addLine("DEBUG: Starting autonomous sequence");
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

        // Initialize pure pursuit controller for driving
        telemetry.addLine("Initializing pure pursuit controller...");
        telemetry.update();
        purePursuitController = new PurePursuitController();
        purePursuitController.setParameters(12.0, DRIVE_SPEED * SixWheelDriveController.VelocityParams.MAX_TICKS_PER_SEC, 15.0); // look ahead, max velocity (ticks/sec), track width

        // Initialize shooter components
        telemetry.addLine("Initializing shooter components...");
        telemetry.update();
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        transferController = new TransferController(this);
        rampController = new RampController(this);

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

        // Initialize ball alignment controller - passes opMode which initializes its own drive controller
        try {
            telemetry.addLine("Initializing ball alignment controller...");
            telemetry.update();
            ballAlignmentController = new BallAlignmentController(this);
            telemetry.addLine("✓ Ball alignment controller initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Ball alignment failed to init: " + e.getMessage());
            ballAlignmentController = null;
        }

        // Initialize auto shoot controller
        telemetry.addLine("Initializing auto shoot controller...");
        telemetry.update();
        autoShootController = new AutoShootController(
                this,
                driveController,
                shooterController,
                intakeController,
                transferController,
                limelightController
        );

        telemetry.addLine("All controllers initialized successfully");
        telemetry.update();
    }

    private void executeAutonomousSequence() throws InterruptedException {
        // Step 1: Start shooter immediately (moved from initialization)
        telemetry.clear();
        telemetry.addLine("=== STEP 1: STARTING SHOOTER ===");
        telemetry.addData("Target RPM", 2350);
        telemetry.update();

        shooterController.setShooterRPM(2800);

        // Wait for shooter to reach RPM
        long rpmStartTime = System.currentTimeMillis();
        while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                (System.currentTimeMillis() - rpmStartTime) < 3000) {
            shooterController.updatePID();
            driveController.updateOdometry();

            telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Target RPM", 2800);
            telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
            telemetry.update();

            sleep(20);
        }

        telemetry.addLine("Shooter at target RPM");
        telemetry.update();

        // Step 2: Drive backward 60 inches using odometry only
        telemetry.clear();
        telemetry.addLine("=== STEP 2: DRIVING BACKWARD ===");
        telemetry.addData("Target Distance", "%.1f inches", DRIVE_DISTANCE_INCHES);
        telemetry.update();

        driveBackwardWithOdometry(DRIVE_DISTANCE_INCHES);

        telemetry.addLine("Backward drive complete");
        telemetry.update();
        sleep(500);

        // Step 3: Limelight alignment and shooting
        telemetry.clear();
        telemetry.addLine("=== STEP 3: ALIGNMENT & SHOOTING ===");
        telemetry.addData("Time", "%.1f seconds", runtime.seconds());
        telemetry.addLine("Starting auto shoot sequence...");
        telemetry.addLine("DEBUG: About to call executeDistanceBasedAutoShoot()");
        telemetry.update();

        // Execute auto shoot sequence (includes alignment and shooting)
        telemetry.clear();
        telemetry.addLine("=== AUTO SHOOT DEBUG ===");
        telemetry.addLine("Calling executeDistanceBasedAutoShoot()...");
        telemetry.addData("Limelight Available", limelightController != null ? "YES" : "NO");
        telemetry.addData("AutoShoot Controller Available", autoShootController != null ? "YES" : "NO");
        telemetry.update();
        sleep(1000);

        autoShootController.executeDistanceBasedAutoShoot();

        // Wait for auto shoot to complete (keep shooter running)
        telemetry.clear();
        telemetry.addLine("=== WAITING FOR AUTO SHOOT DEBUG ===");
        telemetry.addLine("Waiting for auto shoot to complete...");
        telemetry.addData("AutoShoot Active", autoShootController.isAutoShooting() ? "YES" : "NO");
        telemetry.addData("AutoShoot Status", autoShootController.getCurrentStatus());
        telemetry.addData("Shooter RPM", "%.0f", shooterController.getShooterRPM());
        telemetry.addData("Target RPM", 2800);
        telemetry.update();
        sleep(1000);

        long shootStartTime = System.currentTimeMillis();
        boolean autoShootCompleted = false;
        int waitLoopCount = 0;

        while (opModeIsActive() && autoShootController.isAutoShooting() &&
                 (System.currentTimeMillis() - shootStartTime) < 10000) {  // Increased timeout
            shooterController.updatePID();
            driveController.updateOdometry();
            waitLoopCount++;

            telemetry.clear();
            telemetry.addLine("=== WAITING FOR AUTO SHOOT ===");
            telemetry.addData("Loop Count", waitLoopCount);
            telemetry.addData("AutoShoot Status", autoShootController.isAutoShooting() ? "ACTIVE" : "IDLE");
            telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
            telemetry.addData("Shooter RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Target RPM", 2800);
            telemetry.addData("Wait Time", "%.1f seconds", (System.currentTimeMillis() - shootStartTime) / 1000.0);
            telemetry.addData("AutoShoot Status Detail", autoShootController.getCurrentStatus());
            telemetry.addData("Limelight Has Target", limelightController != null && limelightController.hasTarget() ? "YES" : "NO");
            if (limelightController != null) {
                telemetry.addData("Limelight Error", "%.2f°", limelightController.getTargetError());
                telemetry.addData("Limelight State", limelightController.getState().toString());
            }
            telemetry.update();

            sleep(20);
        }

        if (autoShootController.isAutoShooting()) {
            telemetry.addLine("⚠️ Auto shoot timed out!");
            telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
            telemetry.update();
            sleep(1000);
        } else {
            telemetry.addLine("✅ Auto shoot completed");
            telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
            telemetry.update();
            sleep(500);
        }

        // Check if we have ball alignment controller
        telemetry.clear();
        telemetry.addLine("=== TRANSITION DEBUG ===");
        telemetry.addData("AutoShoot Active", autoShootController.isAutoShooting() ? "YES" : "NO");
        telemetry.addData("AutoShoot Status", autoShootController.getCurrentStatus());
        telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
        telemetry.update();
        sleep(1000);

        if (ballAlignmentController != null) {
            telemetry.clear();
            telemetry.addLine("=== STEP 4: BALL ALIGNMENT ===");
            telemetry.addData("Ball Controller Status", "AVAILABLE");
            telemetry.addData("Proceeding to ball alignment", "after auto shoot");
            telemetry.addData("Ball Controller hasSeenValidResult", ballAlignmentController.hasSeenValidResult());
            telemetry.update();
        } else {
            telemetry.addLine("⚠️ No ball alignment controller - skipping to ball collection");
            telemetry.update();
        }

        // Stop shooter after shooting
        telemetry.addLine("Stopping shooter...");
        shooterController.shooterStop();
        sleep(500);

        // Step 5: Ball Collection Sequence
        telemetry.clear();
        telemetry.addLine("=== STEP 5: BALL COLLECTION ===");
        telemetry.addData("Time", "%.1f seconds", runtime.seconds());
        telemetry.update();

        if (ballAlignmentController != null) {
            telemetry.addLine("Starting ball collection sequence...");
            telemetry.update();
            executeBallCollectionSequence();
        } else {
            telemetry.addLine("⚠️ Ball alignment controller not available");
            telemetry.addLine("Skipping ball collection");
            telemetry.update();
            sleep(2000);
        }

        // Final status
        telemetry.clear();
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Total Runtime", "%.2f seconds", runtime.seconds());
        telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
        telemetry.addLine("Robot stopped");
        telemetry.update();

        // Keep robot stopped
        while (opModeIsActive() && !isStopRequested()) {
            driveController.stopDrive();
            sleep(50);
        }
    }

    /**
     * Execute the ball collection sequence:
     * 1. Turn left to search area
     * 2. Search for and align with ball
     * 3. Start intake
     * 4. Drive forward to collect ball
     * 5. Stop intake
     */
    private void executeBallCollectionSequence() throws InterruptedException {
        telemetry.addLine("=== BALL COLLECTION SEQUENCE STARTED ===");
        telemetry.update();
        sleep(500);

        // Sub-step 5a: Turn left to begin search
        telemetry.clear();
        telemetry.addLine("=== 5a: TURNING TO SEARCH AREA ===");
        telemetry.addData("Turn Angle", "%.1f degrees", BALL_SEARCH_TURN_DEGREES);
        telemetry.addData("Time", "%.1f seconds", runtime.seconds());
        telemetry.update();

        // Calculate new heading (current + turn amount)
        double currentHeading = Math.toDegrees(driveController.getHeading());
        double targetHeading = currentHeading + BALL_SEARCH_TURN_DEGREES;

        telemetry.addData("Current Heading", "%.1f degrees", currentHeading);
        telemetry.addData("Target Heading", "%.1f degrees", targetHeading);
        telemetry.update();

        // Normalize to 0-360 range
        while (targetHeading < 0) targetHeading += 360;
        while (targetHeading >= 360) targetHeading -= 360;

        telemetry.addData("Normalized Target Heading", "%.1f degrees", targetHeading);
        telemetry.update();

        turnToHeading(targetHeading, TURN_SPEED);

        telemetry.addLine("Turn complete - beginning ball search");
        telemetry.update();
        sleep(500);

        // Sub-step 5b: Search for and align with ball
        telemetry.clear();
        telemetry.addLine("=== 5b: SEARCHING FOR BALL ===");
        telemetry.addData("Timeout", "%.1f seconds", BALL_ALIGNMENT_TIMEOUT_MS / 1000.0);
        telemetry.addData("Time", "%.1f seconds", runtime.seconds());
        telemetry.addLine("DEBUG: Starting ball tracking...");
        telemetry.addData("DEBUG: Ball Controller Active", ballAlignmentController.isTracking() ? "YES" : "NO");
        telemetry.update();
        sleep(500);

        telemetry.addLine("Starting ball tracking...");
        telemetry.update();
        ballAlignmentController.startTracking();

        long alignmentStartTime = System.currentTimeMillis();
        boolean ballAligned = false;

        telemetry.addLine("Entering ball alignment loop...");
        telemetry.update();

        // Alignment loop with timeout
        int loopCount = 0;
        while (opModeIsActive() &&
                 (System.currentTimeMillis() - alignmentStartTime) < BALL_ALIGNMENT_TIMEOUT_MS) {

            driveController.updateOdometry();
            loopCount++;

            telemetry.clear();
            telemetry.addLine("=== BALL SEARCH DEBUG ===");
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Ball Controller State", ballAlignmentController.getState().toString());
            telemetry.addData("Has Ball", ballAlignmentController.hasBall() ? "YES" : "NO");
            telemetry.addData("Is Aligned", ballAlignmentController.isAligned() ? "YES" : "NO");
            telemetry.addData("Ball Distance", "%.1f in", ballAlignmentController.getBallDistance());
            telemetry.addData("Ball Area", "%.0f px", ballAlignmentController.getBallArea());
            telemetry.addData("Ball Pixel X", "%.0f", ballAlignmentController.getBallPixelX());
            telemetry.addData("Ball Color", ballAlignmentController.getBallColor());
            telemetry.addData("Motor Output", "%.3f", ballAlignmentController.getMotorOutput());
            telemetry.addData("Has Seen Valid Result", ballAlignmentController.hasSeenValidResult() ? "YES" : "NO");

            telemetry.addLine("--- Calling ballAlignmentController.align() ---");
            ballAlignmentController.align();  // This handles searching and aligning

            // Display ball alignment telemetry
            ballAlignmentController.displayTelemetry();
            telemetry.addData("Time Elapsed", "%.1fs",
                    (System.currentTimeMillis() - alignmentStartTime) / 1000.0);
            telemetry.addData("Alignment Loop Iteration", loopCount);
            telemetry.update();

            // Check if we're aligned
            if (ballAlignmentController.isAligned()) {
                ballAligned = true;
                telemetry.addLine("✅ BALL ALIGNED!");
                telemetry.addData("Time to Align", "%.1fs", (System.currentTimeMillis() - alignmentStartTime) / 1000.0);
                telemetry.update();
                break;
            }

            sleep(20);
        }

        telemetry.addLine("Exiting ball alignment loop...");
        telemetry.update();
        ballAlignmentController.stopTracking();

        if (!ballAligned) {
            telemetry.clear();
            telemetry.addLine("⚠️ Ball alignment timeout");
            telemetry.addData("Time Spent", "%.1fs", (System.currentTimeMillis() - alignmentStartTime) / 1000.0);
            telemetry.addLine("Skipping ball collection");
            telemetry.update();
            sleep(2000);
            return;
        }

        sleep(500);

        // Sub-step 5c: Start intake and drive forward to collect ball
        telemetry.clear();
        telemetry.addLine("=== 5c: COLLECTING BALL ===");
        telemetry.addData("Distance", "%.1f inches", BALL_COLLECTION_DISTANCE);
        telemetry.update();

        // Start intake
        intakeController.intakeFull();

        // Drive forward while intake is running
        driveForwardWithIntake(BALL_COLLECTION_DISTANCE, BALL_COLLECTION_SPEED);

        // Stop intake
        intakeController.intakeStop();

        telemetry.clear();
        telemetry.addLine("=== BALL COLLECTION COMPLETE ===");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drive forward a specified distance while intake is running
     */
    private void driveForwardWithIntake(double distance, double speed) throws InterruptedException {
        double startX = driveController.getX();
        double startY = driveController.getY();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 8000; // 8 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Calculate distance traveled
            double currentX = driveController.getX();
            double currentY = driveController.getY();
            double distanceTraveled = Math.hypot(currentX - startX, currentY - startY);

            // Check if we've reached the target distance
            if (distanceTraveled >= distance) {
                break;
            }

            // Drive forward
            driveController.tankDrive(speed, speed);

            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
            telemetry.addData("Intake", "ACTIVE");
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Drive to a target position using pure pursuit algorithm
     */
    private void driveToPositionUsingPurePursuit(double distance) throws InterruptedException {
        // Set target position relative to current position
        double startX = driveController.getX();
        double startY = driveController.getY();
        double startHeading = driveController.getHeading();

        telemetry.addData("DEBUG - Start X", "%.2f", startX);
        telemetry.addData("DEBUG - Start Y", "%.2f", startY);
        telemetry.addData("DEBUG - Start Heading", "%.2f", Math.toDegrees(startHeading));
        telemetry.addData("DEBUG - Distance", "%.2f", distance);
        telemetry.update();

        Vector2d targetPosition = new Vector2d(startX + distance, startY); // Move in X direction by specified distance (negative distance = backward)
        purePursuitController.setTargetPosition(targetPosition);

        telemetry.addData("DEBUG - Target X", "%.2f", targetPosition.x);
        telemetry.addData("DEBUG - Target Y", "%.2f", targetPosition.y);
        telemetry.update();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 10000; // 10 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Get current pose
            Pose2d currentPose = new Pose2d(
                    driveController.getX(),
                    driveController.getY(),
                    driveController.getHeading()
            );

            // Update pure pursuit
            double[] powers = purePursuitController.update(currentPose);
            driveController.tankDriveVelocityNormalized(powers[0], powers[1]);

            // Check if we're at the target
            double distToEnd = Math.hypot(currentPose.position.x - targetPosition.x, currentPose.position.y - targetPosition.y);
            if (distToEnd < POSITION_TOLERANCE_INCHES) {
                break;
            }

            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Distance Remaining", "%.2f inches", distToEnd);
            telemetry.addData("Current X", "%.2f", currentPose.position.x);
            telemetry.addData("Current Y", "%.2f", currentPose.position.y);
            telemetry.addData("Left Power", "%.3f", powers[0]);
            telemetry.addData("Right Power", "%.3f", powers[1]);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Turn to a target heading using odometry
     */
    private void turnToHeading(double targetHeadingDegrees, double speed) throws InterruptedException {
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);

        long turnStartTime = System.currentTimeMillis();
        long maxTurnTimeMs = 5000; // 5 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - turnStartTime) < maxTurnTimeMs) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = angleDifference(currentHeading, targetHeadingRadians);

            // Convert to degrees for display
            double headingErrorDegrees = Math.toDegrees(headingError);

            // Check if we're close enough to target heading
            if (Math.abs(headingErrorDegrees) <= HEADING_TOLERANCE_DEGREES) {
                break;
            }

            // Simple proportional turn with safety check
            double turnPower = 0.0;
            if (Math.abs(headingErrorDegrees) > 0.1) { // Prevent division by very small numbers
                turnPower = speed * Math.min(1.0, Math.abs(headingErrorDegrees) / 30.0);
            }
            turnPower = Math.copySign(turnPower, headingError);

            driveController.tankDrive(-turnPower, turnPower);

            telemetry.addData("Heading Error", "%.2f degrees", headingErrorDegrees);
            telemetry.addData("Current Heading", "%.2f degrees", Math.toDegrees(currentHeading));
            telemetry.addData("Target Heading", "%.2f degrees", targetHeadingDegrees);
            telemetry.addData("Turn Power", "%.3f", turnPower);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Drive backward a specified distance using odometry only (no pure pursuit)
     */
    private void driveBackwardWithOdometry(double distance) throws InterruptedException {
        double startX = driveController.getX();
        double startY = driveController.getY();

        long driveStartTime = System.currentTimeMillis();
        long maxDriveTimeMs = 8000; // 8 second timeout

        while (opModeIsActive() && (System.currentTimeMillis() - driveStartTime) < maxDriveTimeMs) {
            driveController.updateOdometry();

            // Calculate distance traveled (backward movement)
            double currentX = driveController.getX();
            double currentY = driveController.getY();
            double distanceTraveled = Math.abs(startX - currentX); // Assuming backward movement changes Y coordinate negatively

            // Check if we've reached the target distance
            if (distanceTraveled >= distance) {
                break;
            }

            // Drive straight backward
            driveController.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);

            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Remaining", "%.2f inches", distance - distanceTraveled);
            telemetry.addData("Current Y", "%.2f", currentY);
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Calculate the smallest angle difference between two angles (in radians)
     */
    private double angleDifference(double current, double target) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}