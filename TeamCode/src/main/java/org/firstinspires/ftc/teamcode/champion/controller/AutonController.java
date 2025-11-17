package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * STABLE Controller for autonomous - minimal telemetry, single thread
 */
@Config
public class AutonController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final TransferController transferController;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final LimelightAlignmentController limelightController;
    private final AutoShootController autoShootController;

    // ========== MOVEMENT PID ==========
    @Config
    public static class MovementPID {
        public static double kP = 0.046;
        public static double kD = 0.017;
        public static double MIN_SPEED = 0.05;
        public static double MAX_SPEED = 0.65;
    }

    // ========== TURN PID ==========
    @Config
    public static class TurnPID {
        public static double kP = 0.65;
        public static double kD = 0.04;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
    }

    // Configuration
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static long SHOOT_DURATION = 1400;
    public static double HEADING_THRESHOLD_DEG = 2;

    // Single PID thread
    private Thread pidThread;
    private volatile boolean runPid = false;

    // Simplified intake control (no thread)
    private double intakePower = 0.0;

    // Single timer for efficiency
    private final ElapsedTime timer = new ElapsedTime();

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

    // ========== SIMPLIFIED THREAD MANAGEMENT ==========

    /**
     * Start single PID thread
     */
    public void startPidUpdateThread() {
        if (pidThread != null && pidThread.isAlive()) return;

        runPid = true;
        pidThread = new Thread(() -> {
            while (runPid && opMode.opModeIsActive()) {
                shooterController.updatePID();
                try {
                    Thread.sleep(20); // 50Hz
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        pidThread.setPriority(Thread.MAX_PRIORITY);
        pidThread.start();
    }

    public void stopPidUpdateThread() {
        runPid = false;
        if (pidThread != null) {
            try {
                pidThread.interrupt();
            } catch (Exception e) {
                // Ignore
            }
        }
    }

    /**
     * NO INTAKE THREAD - Direct control only
     */
    public void startIntakeThread() {
        // Empty - no thread needed
    }

    public void stopIntakeThread() {
        // Empty - no thread needed
        intakeController.intakeStop();
    }

    public void setIntakePower(double power) {
        if (power > 0.5) {
            intakeController.intakeFull();
        } else if (power < -0.5) {
            intakeController.intakeEject();
        } else {
            intakeController.intakeStop();
        }
    }

    // ========== SIMPLIFIED MOVEMENT ==========

    /**
     * Move robot - MINIMAL telemetry
     */
    public void moveRobot(double distanceInches, double maxSpeed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        double lastError = 0.0;
        timer.reset();

        // Single telemetry at start
        opMode.telemetry.addData("Moving", "%.1f inches", distanceInches);
        opMode.telemetry.update();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double error = targetDistance - currentDistance;

            // Exit if complete
            if (error < 0.5 || currentDistance >= targetDistance) {
                break;
            }

            // Simple P-D control
            double pTerm = MovementPID.kP * error;
            double dTerm = MovementPID.kD * (error - lastError);
            lastError = error;

            double speed = pTerm + dTerm;
            speed = Math.max(MovementPID.MIN_SPEED, Math.min(maxSpeed, Math.abs(speed)));
            speed = Math.min(speed, MovementPID.MAX_SPEED) * direction;

            driveController.tankDriveVelocityNormalized(speed, speed);

            // NO telemetry in loop!
            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    /**
     * Turn to heading - MINIMAL telemetry
     */
    public void turnToHeading(double targetDegrees) {
        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        double lastError = 0.0;
        timer.reset();

        // Single telemetry at start
        opMode.telemetry.addData("Turning", "%.1f°", targetDegrees);
        opMode.telemetry.update();

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double headingError = normalizeAngle(targetRad - driveController.getHeading());

            // Exit if within threshold
            if (Math.abs(headingError) < thresholdRad) break;

            // Simple P-D control
            double pTerm = TurnPID.kP * headingError;
            double dTerm = TurnPID.kD * (headingError - lastError);
            lastError = headingError;

            double power = pTerm + dTerm;

            // Apply limits
            double absPower = Math.abs(power);
            if (absPower < TurnPID.MIN_POWER) {
                power = Math.signum(power) * TurnPID.MIN_POWER;
            } else if (absPower > TurnPID.MAX_POWER) {
                power = Math.signum(power) * TurnPID.MAX_POWER;
            }

            driveController.tankDrive(-power, power);

            // NO telemetry in loop!
            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    public double getCurrentHeading() {
        driveController.updateOdometry();
        return Math.toDegrees(driveController.getHeading());
    }

    // ========== SIMPLIFIED SHOOTING ==========

    /**
     * Quick shoot - NO telemetry spam
     */
    public void quickShoot() {
        // Wait for RPM (max 300ms)
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 300) {
            if (Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 150) {
                break;
            }
            opMode.sleep(20);
        }

        // Optional quick alignment
        if (limelightController != null) {
            try {
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                timer.reset();
                while (opMode.opModeIsActive() && timer.milliseconds() < 500) {
                    limelightController.align(AutoShootController.APRILTAG_ID);
                    if (limelightController.getTargetError() <= 1.5) break;
                    opMode.sleep(20);
                }

                limelightController.stopAlignment();
                driveController.stopDrive();
            } catch (Exception e) {
                // Continue without alignment
            }
        }

        // Shoot
        transferController.transferFull();
        intakeController.intakeFull();

        // Single telemetry
        opMode.telemetry.addLine("Shooting!");
        opMode.telemetry.update();

        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < SHOOT_DURATION) {
            opMode.sleep(50); // Less frequent checks
        }

        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * SIMPLIFIED warmup - no threading
     */
    public void warmupShooter() {
        // Just wait a bit for RPM to stabilize
        timer.reset();
        while (timer.milliseconds() < 500) {
            if (Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 150) {
                break;
            }
            opMode.sleep(50);
        }
    }

    // ========== PATTERN DETECTION ==========

    /**
     * Detect pattern - NO telemetry spam
     */
    public int detectPattern() {
        if (autoShootController == null) return 0;

        try {
            // Brief wait for camera
            opMode.sleep(300);

            int tagId = autoShootController.getVisibleAprilTagId();

            // Mapping: 21->GPP(2), 22->PGP(1), 23->PPG(0)
            if (tagId == 21) return 2;
            if (tagId == 22) return 1;
            if (tagId == 23) return 0;
        } catch (Exception e) {
            // Use default
        }
        return 0; // Default PPG
    }

    /**
     * Simple sleep
     */
    public void sleepWithPid(long milliseconds) {
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < milliseconds) {
            opMode.sleep(20);
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}