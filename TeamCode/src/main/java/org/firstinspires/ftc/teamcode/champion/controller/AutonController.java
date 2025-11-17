package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * Controller class for autonomous movement operations with PID control
 * Handles movement, turning, shooting, and pattern detection
 */
@Config
public class AutonController {

    // Reference to parent OpMode
    private final LinearOpMode opMode;

    // Controller references
    private final SixWheelDriveController driveController;
    private final TransferController transferController;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final LimelightAlignmentController limelightController;
    private final AutoShootController autoShootController;

    // Configuration parameters
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double TURN_POWER = 0.25;
    public static long SHOOT_DURATION = 1400;
    public static long SHOOTER_WARMUP = 800;
    public static long ALIGNMENT_TIMEOUT = 750;
    public static double ALIGNMENT_THRESHOLD = 1.5;
    public static double HEADING_THRESHOLD_DEG = 2;
    public static long PID_UPDATE_INTERVAL = 5;

    // Thread management
    private Thread pidUpdateThread;
    private volatile boolean pidThreadRunning = false;
    private boolean shooterReady = false;

    /**
     * Constructor
     */
    public AutonController(
            LinearOpMode opMode,
            SixWheelDriveController driveController,
            TransferController transferController,
            ShooterController shooterController,
            IntakeController intakeController,
            LimelightAlignmentController limelightController,
            AutoShootController autoShootController) {

        this.opMode = opMode;
        this.driveController = driveController;
        this.transferController = transferController;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.limelightController = limelightController;
        this.autoShootController = autoShootController;
    }

    // ========== THREAD MANAGEMENT ==========

    /**
     * Start background thread for continuous PID updates
     */
    public void startPidUpdateThread() {
        pidThreadRunning = true;
        pidUpdateThread = new Thread(() -> {
            while (pidThreadRunning && opMode.opModeIsActive()) {
                try {
                    shooterController.updatePID();
                    Thread.sleep(PID_UPDATE_INTERVAL);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        pidUpdateThread.setPriority(Thread.MAX_PRIORITY);
        pidUpdateThread.start();
    }

    /**
     * Stop the PID update thread
     */
    public void stopPidUpdateThread() {
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

    // ========== MOVEMENT METHODS ==========

    /**
     * Move robot using odometry-based distance tracking
     */
    public void moveRobot(double distanceInches, double speed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startY = driveController.getY();
        double direction = Math.signum(distanceInches);

        ElapsedTime moveTimer = new ElapsedTime();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(direction * speed, direction * speed);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double deltaX = driveController.getX() - startX;
            double deltaY = driveController.getY() - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Enhanced telemetry
            opMode.telemetry.addData("Distance Traveled", String.format("%.2f", distanceTraveled));
            opMode.telemetry.addData("Target Distance", String.format("%.2f", Math.abs(distanceInches)));
            opMode.telemetry.addData("Delta X", String.format("%.2f", deltaX));
            opMode.telemetry.addData("Delta Y", String.format("%.2f", deltaY));
            opMode.telemetry.addData("Current Heading", String.format("%.1f°", Math.toDegrees(driveController.getHeading())));
            opMode.telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
            opMode.telemetry.addData("Time Elapsed", String.format("%.1fs", moveTimer.seconds()));
            opMode.telemetry.update();

            if (distanceTraveled >= Math.abs(distanceInches)) {
                break;
            }
            opMode.sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Turn to target heading
     */
    public void turnToHeading(double targetDegrees) {
        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);
        double turnPower = 0.25;

        ElapsedTime turnTimer = new ElapsedTime();

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double error = normalizeAngle(targetRad - driveController.getHeading());
            if (Math.abs(error) < thresholdRad) break;

            double power = Math.signum(error) * turnPower;
            driveController.tankDrive(-power, power);

            // Telemetry during turn
            if (turnTimer.milliseconds() % 100 < 10) {
                opMode.telemetry.addData("Turn Error", Math.toDegrees(error));
                opMode.telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                opMode.telemetry.update();
            }

            opMode.sleep(5);
        }
        driveController.stopDrive();
    }

    // ========== SHOOTING METHODS ==========

    /**
     * Ultra-fast shooting sequence
     */
    public void quickShoot() {
        // Quick RPM check (max 400ms wait)
        ElapsedTime rpmWait = new ElapsedTime();
        while (opMode.opModeIsActive() && !shooterReady && rpmWait.milliseconds() < 400) {
            shooterReady = Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 100;

            if (rpmWait.milliseconds() % 100 < 10) {
                opMode.telemetry.addData("Waiting for RPM", shooterController.getShooterRPM());
                opMode.telemetry.update();
            }
            opMode.sleep(10);
        }

        // Quick alignment if Limelight available
        if (limelightController != null) {
            try {
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                ElapsedTime alignTime = new ElapsedTime();
                while (opMode.opModeIsActive() && alignTime.milliseconds() < ALIGNMENT_TIMEOUT) {
                    limelightController.align(AutoShootController.APRILTAG_ID);

                    if (limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) break;
                    opMode.sleep(10);
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
        while (opMode.opModeIsActive() && shootTime.milliseconds() < SHOOT_DURATION) {
            if (shootTime.milliseconds() % 100 < 10) {
                opMode.telemetry.addData("Shooting RPM", shooterController.getShooterRPM());
                opMode.telemetry.update();
            }
            opMode.sleep(5);
        }

        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * Background shooter warmup
     */
    public void warmupShooter() {
        ElapsedTime warmup = new ElapsedTime();
        while (warmup.milliseconds() < SHOOTER_WARMUP) {
            // Check if we're close enough
            if (warmup.milliseconds() > 500) {
                double rpm = shooterController.getShooterRPM();
                if (Math.abs(rpm - CONSTANT_SHOOTER_RPM) < 100) {
                    break;
                }
            }
            opMode.sleep(10);
        }
        shooterReady = true;
    }

    // ========== PATTERN DETECTION ==========

    /**
     * Fast pattern detection
     * Updated mapping: ID21→GPP, ID22→PGP, ID23→PPG
     * @return pattern index (0=PPG, 1=PGP, 2=GPP)
     */
    public int detectPattern() {
        if (autoShootController == null) return 0; // Default to PPG

        try {
            ElapsedTime waitTimer = new ElapsedTime();
            while (opMode.opModeIsActive() && waitTimer.milliseconds() < 500) {
                if (waitTimer.milliseconds() % 100 < 10) {
                    opMode.telemetry.addData("Detecting pattern...", "");
                    opMode.telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                    opMode.telemetry.update();
                }
                opMode.sleep(10);
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

    // ========== UTILITY METHODS ==========

    /**
     * Sleep while ensuring responsiveness
     */
    public void sleepWithPid(long milliseconds) {
        ElapsedTime sleepTimer = new ElapsedTime();
        while (opMode.opModeIsActive() && sleepTimer.milliseconds() < milliseconds) {
            opMode.sleep(10);
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

    /**
     * Reset shooter ready state
     */
    public void resetShooterReady() {
        shooterReady = false;
    }

    /**
     * Check if shooter is ready
     */
    public boolean isShooterReady() {
        return shooterReady;
    }
}