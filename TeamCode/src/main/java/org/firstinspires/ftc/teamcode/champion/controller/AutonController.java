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

    // ========== MOVEMENT PID COEFFICIENTS ==========
    @Config
    public static class MovementPID {
        public static double kP = 0.046;
        public static double kI = 0.0;
        public static double kD = 0.017;
        public static double MIN_SPEED = 0.05;
        public static double MAX_SPEED = 0.65;
    }

    // ========== TURN PID COEFFICIENTS ==========
    @Config
    public static class TurnPID {
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.04;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
    }

    // Configuration parameters
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static long SHOOT_DURATION = 1400;
    public static long SHOOTER_WARMUP = 800;
    public static long ALIGNMENT_TIMEOUT = 500;
    public static double ALIGNMENT_THRESHOLD = 1.5;
    public static double HEADING_THRESHOLD_DEG = 2;
    public static long PID_UPDATE_INTERVAL = 5;

    // Thread management
    private Thread pidUpdateThread;
    private volatile boolean pidThreadRunning = false;

    private Thread intakeThread;
    private volatile boolean intakeThreadRunning = false;
    private volatile double intakeTargetPower = 0.0;

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
                shooterController.updatePID();
                opMode.sleep(PID_UPDATE_INTERVAL);
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

    /**
     * Start background thread for continuous intake control
     */
    public void startIntakeThread() {
        intakeThreadRunning = true;
        intakeThread = new Thread(() -> {
            while (intakeThreadRunning && opMode.opModeIsActive()) {
                if (intakeTargetPower != 0.0) {
                    if (intakeTargetPower > 0) {
                        intakeController.intakeFull();
                    } else {
                        intakeController.intakeStop();
                    }
                }
                opMode.sleep(10);
            }
        });
        intakeThread.setPriority(Thread.NORM_PRIORITY);
        intakeThread.start();
    }

    /**
     * Stop the intake thread
     */
    public void stopIntakeThread() {
        intakeThreadRunning = false;
        intakeTargetPower = 0.0;
        if (intakeThread != null) {
            try {
                intakeThread.interrupt();
                intakeThread.join(100);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
    }

    /**
     * Set intake power via thread (non-blocking)
     */
    public void setIntakePower(double power) {
        intakeTargetPower = power;
    }

    // ========== MOVEMENT METHODS ==========

    /**
     * Move robot with PID control for smooth acceleration and deceleration
     */
    public void moveRobot(double distanceInches, double maxSpeed) {
        // Reset PID state
        // PID state variables for movement
        double movementIntegral = 0.0;
        double movementLastError = 0.0;

        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double distanceError = targetDistance - currentDistance;

            // Calculate PID output
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Proportional term
            double pTerm = MovementPID.kP * distanceError;

            // Integral term (with anti-windup)
            movementIntegral += distanceError * dt;
            movementIntegral = Math.max(-10, Math.min(10, movementIntegral));
            double iTerm = MovementPID.kI * movementIntegral;

            // Derivative term
            double dTerm = 0.0;
            if (dt > 0) {
                dTerm = MovementPID.kD * (distanceError - movementLastError) / dt;
            }
            movementLastError = distanceError;

            // Calculate speed with PID
            double speed = pTerm + iTerm + dTerm;

            // Apply speed limits
            speed = Math.max(MovementPID.MIN_SPEED, Math.min(maxSpeed, Math.abs(speed)));
            speed = Math.min(speed, MovementPID.MAX_SPEED);

            // Apply direction
            speed *= direction;

            driveController.tankDriveVelocityNormalized(speed, speed);

            // Telemetry updates
            if (moveTimer.milliseconds() % 100 < 10) {
                opMode.telemetry.addData("Distance Error", String.format(Locale.US, "%.2f in", distanceError));
                opMode.telemetry.addData("Movement Speed", String.format(Locale.US, "%.2f", speed));
                opMode.telemetry.addData("Intake Running", intakeTargetPower > 0 ? "YES" : "NO");
                opMode.telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                opMode.telemetry.addData("Target RPM", CONSTANT_SHOOTER_RPM);
                opMode.telemetry.update();
            }

            // Exit condition
            if (distanceError < 0.5 || currentDistance >= targetDistance) {
                break;
            }
            opMode.sleep(5);
        }
        driveController.stopDrive();
    }

    /**
     * Turn with PID control for smooth rotation
     */
    public void turnToHeading(double targetDegrees) {
        // Reset PID state
        // PID state variables for turning
        double turnIntegral = 0.0;
        double turnLastError = 0.0;

        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double headingError = normalizeAngle(targetRad - driveController.getHeading());

            // Exit if within threshold
            if (Math.abs(headingError) < thresholdRad) break;

            // Calculate PID output
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Proportional term
            double pTerm = TurnPID.kP * headingError;

            // Integral term (with anti-windup)
            turnIntegral += headingError * dt;
            turnIntegral = Math.max(-5, Math.min(5, turnIntegral));
            double iTerm = TurnPID.kI * turnIntegral;

            // Derivative term
            double dTerm = 0.0;
            if (dt > 0) {
                dTerm = TurnPID.kD * (headingError - turnLastError) / dt;
            }
            turnLastError = headingError;

            // Calculate turn power with PID
            double power = pTerm + iTerm + dTerm;

            // Apply power limits and minimum power
            double absPower = Math.abs(power);
            if (absPower < TurnPID.MIN_POWER) {
                power = Math.signum(power) * TurnPID.MIN_POWER;
            } else if (absPower > TurnPID.MAX_POWER) {
                power = Math.signum(power) * TurnPID.MAX_POWER;
            }

            driveController.tankDrive(-power, power);

            // Telemetry during turn
            if (turnTimer.milliseconds() % 100 < 10) {
                opMode.telemetry.addData("Turn Error", String.format(Locale.US, "%.2f°", Math.toDegrees(headingError)));
                opMode.telemetry.addData("Turn Power", String.format(Locale.US, "%.2f", power));
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
        // Quick RPM check (max 200ms wait)
        ElapsedTime rpmWait = new ElapsedTime();
        while (opMode.opModeIsActive() && !shooterReady && rpmWait.milliseconds() < 200) {
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
