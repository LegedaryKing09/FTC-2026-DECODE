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
@Autonomous(name = "Basic Auton Fixed", group = "Competition")
public class BasicAuton extends LinearOpMode {
    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;

    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double CONSTANT_RAMP_ANGLE = 121.0;
    public static double BACKWARD_DISTANCE = 50.0;
    public static double BASE_FORWARD_DISTANCE = 35.0;
    public static double BASE_REPOSITION_DISTANCE = 20.0;
    public static double MOVEMENT_SPEED = 0.4;
    public static double INTAKE_SPEED = 0.2;
    public static double TURN_POWER = 0.25;
    public static long SHOOT_DURATION = 1400;
    public static long SHOOTER_WARMUP = 800;
    public static long ALIGNMENT_TIMEOUT = 750;
    public static double ALIGNMENT_THRESHOLD = 1.5;
    public static double HEADING_THRESHOLD_DEG = 2;

    // Pattern parameters
    public static double SCAN_ANGLE = -40.0;

    // Pattern-specific parameters (all-in-one)
    public static double[] FETCH_ANGLES = {51.0, 36.0, 26.0};  // PPG, PGP, GPP
    public static double[] EXTRA_DISTANCES = {24.0, 12.0, 0.0}; // PPG, PGP, GPP

    // PID update frequency (milliseconds)
    public static long PID_UPDATE_INTERVAL = 5; // Update every 5ms for consistent control

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime lastPidUpdate = new ElapsedTime();
    private boolean shooterReady = false;

    // Thread for continuous PID updates
    private Thread pidUpdateThread;
    private volatile boolean pidThreadRunning = false;

    @Override
    public void runOpMode() {
        // Fast initialization
        initializeRobot();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();
        pidTimer.reset();

        // Start shooter and ramp
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);

        // Start continuous PID update thread
        startPidUpdateThread();

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        stopPidUpdateThread();
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

        rampController.setAngle(CONSTANT_RAMP_ANGLE);

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

        telemetry.addLine("=== AUTON READY ===");
        telemetry.update();
    }

    /**
     * Start a background thread for continuous PID updates
     * This ensures the shooter maintains RPM even during other operations
     */
    private void startPidUpdateThread() {
        pidThreadRunning = true;
        pidUpdateThread = new Thread(() -> {
            while (pidThreadRunning && opModeIsActive()) {
                try {
                    shooterController.updatePID();
                    Thread.sleep(PID_UPDATE_INTERVAL);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        pidUpdateThread.setPriority(Thread.MAX_PRIORITY); // High priority for consistent updates
        pidUpdateThread.start();
    }

    /**
     * Stop the PID update thread
     */
    private void stopPidUpdateThread() {
        pidThreadRunning = false;
        if (pidUpdateThread != null) {
            try {
                pidUpdateThread.interrupt();
                pidUpdateThread.join(100);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
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

        // PHASE 2: Turn and scan for pattern
        turnToHeading(SCAN_ANGLE);

        // Pattern detection with continuous PID updates
        int patternIndex = detectPattern();

        // PHASE 3: Fetch balls
        double fetchAngle = FETCH_ANGLES[patternIndex];
        double fetchDistance = BASE_FORWARD_DISTANCE + EXTRA_DISTANCES[patternIndex];

        turnToHeading(fetchAngle);

        // Move forward with intake running
        intakeController.intakeFull();
        moveRobot(fetchDistance, INTAKE_SPEED);

        // Brief intake time with PID-aware sleep
        sleepWithPid(400);
        intakeController.intakeStop();

        // PHASE 4: Quick reposition and shoot
        double repositionDistance = BASE_REPOSITION_DISTANCE + EXTRA_DISTANCES[patternIndex];
        moveRobot(-repositionDistance, MOVEMENT_SPEED);

        turnToHeading(0); // Return to original heading
        quickShoot();
    }

    /**
     * Sleep while ensuring PID updates continue (used when PID thread is not active)
     * This is a backup method - the background thread should handle most updates
     */
    private void sleepWithPid(long milliseconds) {
        ElapsedTime sleepTimer = new ElapsedTime();
        while (opModeIsActive() && sleepTimer.milliseconds() < milliseconds) {
            // PID updates are handled by background thread
            // Just sleep in small increments to remain responsive
            sleep(10);
        }
    }

    /**
     * Unified movement method - PID updates handled by background thread
     */
    private void moveRobot(double distanceInches, double speed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double direction = Math.signum(distanceInches);

        ElapsedTime moveTimer = new ElapsedTime();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(direction * speed, direction * speed);

        while (opModeIsActive()) {
            driveController.updateOdometry();

            // Telemetry updates (less frequent to reduce overhead)
            if (moveTimer.milliseconds() % 100 < 10) {
                telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                telemetry.addData("Target RPM", CONSTANT_SHOOTER_RPM);
                telemetry.addData("RPM Error", shooterController.getRPMError());
                telemetry.update();
            }

            if (Math.abs(driveController.getX() - startX) >= Math.abs(distanceInches)) {
                break;
            }
            sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Unified turn method - PID updates handled by background thread
     */
    private void turnToHeading(double targetDegrees) {
        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        ElapsedTime turnTimer = new ElapsedTime();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double error = normalizeAngle(targetRad - driveController.getHeading());
            if (Math.abs(error) < thresholdRad) break;

            double power = Math.signum(error) * TURN_POWER;
            driveController.tankDrive(-power, power);

            // Telemetry during turn
            if (turnTimer.milliseconds() % 100 < 10) {
                telemetry.addData("Turn Error", Math.toDegrees(error));
                telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                telemetry.update();
            }

            sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Ultra-fast shooting sequence
     */
    private void quickShoot() {
        // Quick RPM check (max 600ms wait)
        ElapsedTime rpmWait = new ElapsedTime();
        while (opModeIsActive() && !shooterReady && rpmWait.milliseconds() < 400) {
            shooterReady = Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 100;

            if (rpmWait.milliseconds() % 100 < 10) {
                telemetry.addData("Waiting for RPM", shooterController.getShooterRPM());
                telemetry.update();
            }
            sleep(10);
        }

        // Quick alignment if Limelight available
        if (limelightController != null) {
            try {
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                ElapsedTime alignTime = new ElapsedTime();
                while (opModeIsActive() && alignTime.milliseconds() < ALIGNMENT_TIMEOUT) {
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
            // PID updates handled by background thread
            if (shootTime.milliseconds() % 100 < 10) {
                telemetry.addData("Shooting RPM", shooterController.getShooterRPM());
                telemetry.update();
            }
            sleep(5);
        }

        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * Fast pattern detection
     * Updated mapping: ID21→GPP, ID22→PGP, ID23→PPG
     */
    private int detectPattern() {
        if (autoShootController == null) return 0; // Default to PPG

        try {
            ElapsedTime waitTimer = new ElapsedTime();
            while (opModeIsActive() && waitTimer.milliseconds() < 500) {
                // PID updates handled by background thread
                if (waitTimer.milliseconds() % 100 < 10) {
                    telemetry.addData("Detecting pattern...", "");
                    telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                    telemetry.update();
                }
                sleep(10);
            }
            int tagId = autoShootController.getVisibleAprilTagId();

            // Direct mapping: 21->GPP(2), 22->PGP(1), 23->PPG(0)
            if (tagId == 21) return 2; // GPP
            if (tagId == 22) return 1; // PGP
            if (tagId == 23) return 0; // PPG
        } catch (Exception e) {
            // Use default
        }
        return 0; // Default PPG
    }

    /**
     * Background shooter warmup
     */
    private void warmupShooter() {
        ElapsedTime warmup = new ElapsedTime();
        while (warmup.milliseconds() < SHOOTER_WARMUP) {
            // PID updates handled by background thread

            // Check if we're close enough
            if (warmup.milliseconds() > 500) {
                double rpm = shooterController.getShooterRPM();
                if (Math.abs(rpm - CONSTANT_SHOOTER_RPM) < 100) {
                    break;
                }
            }
            sleep(10);
        }
        shooterReady = true; // Mark ready regardless
    }

    /**
     * Centralized PID update - called from background thread
     * Kept for compatibility but primarily handled by thread
     */
    private void updateShooterPID() {
        shooterController.updatePID();
        pidTimer.reset();
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