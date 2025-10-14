package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;

@Config
@Autonomous(name = "First Auton", group = "Competition")
public class FirstAuton extends LinearOpMode {

    // Configurable parameters
    public static double DRIVE_DISTANCE_INCHES = 60.0;
    public static double TURN_DEGREES = 180.0;
    public static double DRIVE_SPEED = 0.6;
    public static double TURN_SPEED = 0.4;
    public static double POSITION_TOLERANCE_INCHES = 1.0;
    public static double HEADING_TOLERANCE_DEGREES = 2.0;
    public static long POST_TURN_DELAY_MS = 500;

    // Controllers
    private SixWheelDriveController driveController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private TransferController transferController;
    private LimelightAlignmentController limelightController;
    private AutoShootController autoShootController;

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

            waitForStart();
            if (isStopRequested()) return;

            runtime.reset();

            // Execute autonomous sequence
            executeAutonomousSequence();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.update();

            // Log error but don't crash
            sleep(3000);
        }
    }

    private void initializeControllers() throws Exception {
        // Initialize drive controller
        driveController = new SixWheelDriveController(this);

        // Initialize shooter components
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        transferController = new TransferController(this);

        // Initialize Limelight alignment
        limelightController = new LimelightAlignmentController(this);

        // Initialize auto shoot controller
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
        // Step 1: Drive forward 60 inches
        telemetry.clear();
        telemetry.addLine("=== STEP 1: DRIVING FORWARD ===");
        telemetry.addData("Target Distance", "%.1f inches", DRIVE_DISTANCE_INCHES);
        telemetry.update();

        driveToPosition(DRIVE_DISTANCE_INCHES, 0, DRIVE_SPEED);

        telemetry.addLine("Forward drive complete");
        telemetry.update();
        sleep(500);

        // Step 2: Turn 180 degrees
        telemetry.clear();
        telemetry.addLine("=== STEP 2: TURNING 180° ===");
        telemetry.addData("Target Heading", "%.1f degrees", TURN_DEGREES);
        telemetry.update();

        turnToHeading(TURN_DEGREES, TURN_SPEED);

        telemetry.addLine("Turn complete");
        telemetry.update();
        sleep(POST_TURN_DELAY_MS);

        // Step 3: Start shooter at 2350 RPM
        telemetry.clear();
        telemetry.addLine("=== STEP 3: STARTING SHOOTER ===");
        telemetry.addData("Target RPM", 2350);
        telemetry.update();

        shooterController.setShooterRPM(2350);

        // Wait for shooter to reach RPM
        long rpmStartTime = System.currentTimeMillis();
        while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
               (System.currentTimeMillis() - rpmStartTime) < 3000) {
            shooterController.updatePID();
            driveController.updateOdometry();

            telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
            telemetry.addData("Target RPM", 2350);
            telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
            telemetry.update();

            sleep(20);
        }

        telemetry.addLine("Shooter at target RPM");
        telemetry.update();

        // Step 4 & 5: Limelight alignment and shooting
        telemetry.clear();
        telemetry.addLine("=== STEP 4-5: ALIGNMENT & SHOOTING ===");
        telemetry.update();

        // Execute auto shoot sequence (includes alignment and shooting)
        autoShootController.executeAutoShootSequence();

        // Wait for auto shoot to complete
        long shootStartTime = System.currentTimeMillis();
        while (opModeIsActive() && autoShootController.isAutoShooting() &&
               (System.currentTimeMillis() - shootStartTime) < 5000) {
            shooterController.updatePID();
            driveController.updateOdometry();

            telemetry.addData("AutoShoot Status", autoShootController.isAutoShooting() ? "ACTIVE" : "IDLE");
            telemetry.addData("Shots Completed", autoShootController.getShotsCompleted());
            telemetry.update();

            sleep(20);
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
     * Drive to a target position using odometry
     */
    private void driveToPosition(double targetX, double targetY, double speed) throws InterruptedException {
        double startX = driveController.getX();
        double startY = driveController.getY();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentX = driveController.getX();
            double currentY = driveController.getY();

            // Calculate distance to target
            double distanceToTarget = Math.sqrt(
                Math.pow(targetX - (currentX - startX), 2) +
                Math.pow(targetY - (currentY - startY), 2)
            );

            // Check if we're close enough to target
            if (distanceToTarget <= POSITION_TOLERANCE_INCHES) {
                break;
            }

            // Simple proportional drive forward
            double drivePower = speed * Math.min(1.0, distanceToTarget / 10.0);
            driveController.tankDrive(drivePower, drivePower);

            telemetry.addData("Distance to Target", "%.2f inches", distanceToTarget);
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.addData("Current Y", "%.2f", currentY);
            telemetry.addData("Drive Power", "%.3f", drivePower);
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

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = angleDifference(currentHeading, targetHeadingRadians);

            // Convert to degrees for display
            double headingErrorDegrees = Math.toDegrees(headingError);

            // Check if we're close enough to target heading
            if (Math.abs(headingErrorDegrees) <= HEADING_TOLERANCE_DEGREES) {
                break;
            }

            // Simple proportional turn
            double turnPower = speed * Math.min(1.0, Math.abs(headingErrorDegrees) / 30.0);
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
     * Calculate the smallest angle difference between two angles (in radians)
     */
    private double angleDifference(double current, double target) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}