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
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

import java.util.Locale;
@Config
@Autonomous(name = "Basic Auton - Ultra Fast", group = "Competition")
public class BasicAuton extends LinearOpMode {

    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;

    // SPEED OPTIMIZED PARAMETERS
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double CONSTANT_RAMP_ANGLE = 120.0;
    public static double BACKWARD_DISTANCE = 50.0;
    public static double BASE_FORWARD_DISTANCE = 35.0;
    public static double BASE_REPOSITION_DISTANCE = 20.0;
    public static double MOVEMENT_SPEED = 0.4;
    public static double INTAKE_SPEED = 0.3;
    public static double TURN_POWER = 0.1;

    // ULTRA-FAST TIMING
    public static long SHOOT_DURATION = 1400;  // Minimum viable shoot time
    public static long SHOOTER_WARMUP = 600;   // Reduced warmup
    public static long ALIGNMENT_TIMEOUT = 500; // Quick alignment
    public static double ALIGNMENT_THRESHOLD = 1.5; // Less precise but faster
    public static double HEADING_THRESHOLD_DEG = 2; // Less precise turns

    // Pattern parameters
    public static double SCAN_ANGLE = -45.0;

    // Pattern-specific parameters (all-in-one)
    public static double[] FETCH_ANGLES = {51.0, 36.0, 21.0};  // PPG, PGP, GPP
    public static double[] EXTRA_DISTANCES = {24.0, 12.0, 0.0}; // PPG, PGP, GPP

    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime globalTimer = new ElapsedTime();
    private int patternIndex = 0; // 0=PPG, 1=PGP, 2=GPP
    private boolean shooterReady = false;

    @Override
    public void runOpMode() {
        // Fast initialization
        initializeRobot();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();
        pidTimer.reset();

        // Start shooter and ramp IMMEDIATELY
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);
        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        shooterController.shooterStop();
        telemetry.addData("Total Time", String.format(Locale.US, "%.1fs", globalTimer.seconds()));
        telemetry.update();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Try to initialize Limelight (don't fail if not available)
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new AutoShootController(this, driveController, shooterController,
                    intakeController, transferController, limelightController, rampController);
        } catch (Exception e) {
            limelightController = null;
            autoShootController = null;
        }

        telemetry.addLine("=== ULTRA-FAST AUTON READY ===");
        telemetry.update();
    }

    private void executeAutonomousSequence() {
        // PHASE 1: Warmup while moving backward
        Thread warmupThread = new Thread(this::warmupShooter);
        warmupThread.start();

        moveRobot(-BACKWARD_DISTANCE, MOVEMENT_SPEED);

        // Wait for warmup to complete (if not already)
        try {
            warmupThread.join(100); // Max 100ms wait
        } catch (InterruptedException e) {
            // Continue anyway
        }

        // Shoot preloaded balls
        quickShoot();

        // PHASE 2: Turn and scan for pattern (do both simultaneously)
        turnToHeading(SCAN_ANGLE);
        patternIndex = detectPattern(); // Quick pattern detection

        // PHASE 3: Fetch balls
        double fetchAngle = FETCH_ANGLES[patternIndex];
        double fetchDistance = BASE_FORWARD_DISTANCE + EXTRA_DISTANCES[patternIndex];

        turnToHeading(fetchAngle);

        // Move forward with intake running
        intakeController.intakeFull();
        moveRobot(fetchDistance, INTAKE_SPEED);
        sleep(200); // Brief intake time
        intakeController.intakeStop();

        // PHASE 4: Quick reposition and shoot
        double repositionDistance = BASE_REPOSITION_DISTANCE + EXTRA_DISTANCES[patternIndex];
        moveRobot(-repositionDistance, MOVEMENT_SPEED);

        turnToHeading(0); // Return to original heading
        quickShoot();
    }

    /**
     * Unified movement method - handles forward and backward with PID updates
     */
    private void moveRobot(double distanceInches, double speed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double direction = Math.signum(distanceInches);

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(direction * speed, direction * speed);

        while (opModeIsActive()) {
            updateShooterPID();
            driveController.updateOdometry();

            if (Math.abs(driveController.getX() - startX) >= Math.abs(distanceInches)) {
                break;
            }
            sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Unified turn method with PID updates
     */
    private void turnToHeading(double targetDegrees) {
        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        while (opModeIsActive()) {
            updateShooterPID();
            driveController.updateOdometry();

            double error = normalizeAngle(targetRad - driveController.getHeading());
            if (Math.abs(error) < thresholdRad) break;

            double power = Math.signum(error) * TURN_POWER;
            driveController.tankDrive(-power, power);
            sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Ultra-fast shooting sequence - no waiting for perfect conditions
     */
    private void quickShoot() {
        // Quick RPM check (max 200ms wait)
        ElapsedTime rpmWait = new ElapsedTime();
        while (opModeIsActive() && !shooterReady && rpmWait.milliseconds() < 200) {
            updateShooterPID();
            shooterReady = Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 150;
            sleep(10);
        }

        // Quick alignment if Limelight available (max 500ms)
        if (limelightController != null) {
            try {
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                ElapsedTime alignTime = new ElapsedTime();
                while (opModeIsActive() && alignTime.milliseconds() < ALIGNMENT_TIMEOUT) {
                    updateShooterPID();
                    limelightController.align(AutoShootController.APRILTAG_ID);

                    if (limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) break;
                    sleep(10);
                }

                limelightController.stopAlignment();
                driveController.stopDrive();
            } catch (Exception e) {
                // Continue without alignment
            }
        }

        // Shoot immediately
        transferController.transferFull();
        intakeController.intakeFull();

        ElapsedTime shootTime = new ElapsedTime();
        while (opModeIsActive() && shootTime.milliseconds() < SHOOT_DURATION) {
            updateShooterPID();
            sleep(5);
        }

        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * Fast pattern detection - single attempt
     */
    private int detectPattern() {
        if (autoShootController == null) return 2; // Default to PPG

        try {
            sleep(1000); // Brief stabilization
            int tagId = autoShootController.getVisibleAprilTagId();

            // Direct mapping: 21->PPG(0), 22->PGP(1), 23->GPP(2)
            if (tagId >= 21 && tagId <= 23) {
                return tagId - 21;
            }
        } catch (Exception e) {
            // Use default
        }
        return 2; // Default PPG
    }

    /**
     * Background shooter warmup
     */
    private void warmupShooter() {
        ElapsedTime warmup = new ElapsedTime();
        while (warmup.milliseconds() < SHOOTER_WARMUP) {
            updateShooterPID();

            // Check if we're close enough
            if (warmup.milliseconds() > 400) {
                double rpm = shooterController.getShooterRPM();
                if (Math.abs(rpm - CONSTANT_SHOOTER_RPM) < 200) {
                    shooterReady = true;
                    break;
                }
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }
        shooterReady = true; // Mark ready regardless
    }

    /**
     * Centralized PID update - called from all loops
     */
    private void updateShooterPID() {
        if (pidTimer.milliseconds() >= 10) {
            shooterController.updatePID();
            pidTimer.reset();
        }
    }

    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}