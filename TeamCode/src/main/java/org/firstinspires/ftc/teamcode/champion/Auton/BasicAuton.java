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
    public static double BASE_FORWARD_DISTANCE = 0;
    public static double BASE_REPOSITION_DISTANCE = 0;
    public static double MOVEMENT_SPEED = 0.6;
    public static double INTAKE_SPEED = 0.4;
    public static long SHOOT_DURATION = 1400;
    public static long SHOOTER_WARMUP = 800;
    public static long ALIGNMENT_TIMEOUT = 500;
    public static double ALIGNMENT_THRESHOLD = 1.5;
    public static double HEADING_THRESHOLD_DEG = 2;

    // Pattern parameters
    public static double SCAN_ANGLE = -40.0;

    // Pattern-specific parameters (all-in-one)
    public static double[] FETCH_ANGLES = {44.0, 31.0, 26.0};  // PPG, PGP, GPP
    public static double[] EXTRA_DISTANCES = {80.0, 50.0, 20.0}; // PPG, PGP, GPP

    // ========== MOVEMENT PID COEFFICIENTS ==========
    @Config
    public static class MovementPID {
        public static double kP = 0.06;  // Proportional gain for distance error
        public static double kI = 0.0;   // Integral gain
        public static double kD = 0.017;  // Derivative gain
        public static double MIN_SPEED = 0.05;  // Minimum speed to overcome friction
        public static double MAX_SPEED = 0.8;   // Maximum speed cap
    }

    // ========== TURN PID COEFFICIENTS ==========
    @Config
    public static class TurnPID {
        public static double kP = 0.65;   // Proportional gain for heading error
        public static double kI = 0.0;   // Integral gain
        public static double kD = 0.04;  // Derivative gain
        public static double MIN_POWER = 0.15;  // Minimum turn power
        public static double MAX_POWER = 0.65;   // Maximum turn power
    }

    // PID state variables for movement
    private double movementIntegral = 0.0;
    private double movementLastError = 0.0;

    // PID state variables for turning
    private double turnIntegral = 0.0;
    private double turnLastError = 0.0;

    // PID update frequency (milliseconds)
    public static long PID_UPDATE_INTERVAL = 5; // Update every 5ms for consistent control

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();
    private boolean shooterReady = false;

    // Thread for continuous PID updates
    private Thread pidUpdateThread;
    private volatile boolean pidThreadRunning = false;

    // Thread for continuous intake control
    private Thread intakeThread;
    private volatile boolean intakeThreadRunning = false;
    private volatile double intakeTargetPower = 0.0;

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

        // Start continuous intake thread
        startIntakeThread();

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        stopIntakeThread();
        stopPidUpdateThread();
        shooterController.shooterStop();
        intakeController.intakeStop();
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
                shooterController.updatePID();
                sleep(PID_UPDATE_INTERVAL);
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

    /**
     * Start a background thread for continuous intake control
     * This ensures the intake runs smoothly during movement operations
     */
    private void startIntakeThread() {
        intakeThreadRunning = true;
        intakeThread = new Thread(() -> {
            while (intakeThreadRunning && opModeIsActive()) {
                if (intakeTargetPower != 0.0) {
                    // Keep intake running at target power
                    if (intakeTargetPower > 0) {
                        intakeController.intakeFull();
                    } else {
                        intakeController.intakeStop();
                    }
                }
                sleep(10); // Update every 10ms
            }
        });
        intakeThread.setPriority(Thread.NORM_PRIORITY);
        intakeThread.start();
    }

    /**
     * Stop the intake thread
     */
    private void stopIntakeThread() {
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
    private void setIntakePower(double power) {
        intakeTargetPower = power;
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

        // Move forward with intake running (via thread)
        setIntakePower(1.0);  // Start intake via thread
        moveRobot(fetchDistance, INTAKE_SPEED);

        // Brief intake time with PID-aware sleep
        sleepWithPid(400);
        setIntakePower(0.0);  // Stop intake via thread

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
     * Movement with PID control for smooth acceleration and deceleration
     */
    private void moveRobot(double distanceInches, double maxSpeed) {
        // Reset PID state
        movementIntegral = 0.0;
        movementLastError = 0.0;

        driveController.updateOdometry();
        double startX = driveController.getX();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opModeIsActive()) {
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
            movementIntegral = Math.max(-10, Math.min(10, movementIntegral)); // Clamp integral
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

            // Telemetry updates (less frequent to reduce overhead)
            if (moveTimer.milliseconds() % 100 < 10) {
                telemetry.addData("Distance Error", String.format(Locale.US, "%.2f in", distanceError));
                telemetry.addData("Movement Speed", String.format(Locale.US, "%.2f", speed));
                telemetry.addData("Intake Running", intakeTargetPower > 0 ? "YES" : "NO");
                telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                telemetry.addData("Target RPM", CONSTANT_SHOOTER_RPM);
                telemetry.update();
            }

            // Exit condition - within 0.5 inches or past target
            if (distanceError < 0.5 || currentDistance >= targetDistance) {
                break;
            }
            sleep(5);
        }
        driveController.stopDrive();

        // Reset integral for next movement
        movementIntegral = 0.0;
    }

    /**
     * Turn with PID control for smooth rotation
     */
    private void turnToHeading(double targetDegrees) {
        // Reset PID state
        turnIntegral = 0.0;
        turnLastError = 0.0;

        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        while (opModeIsActive()) {
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
            turnIntegral = Math.max(-5, Math.min(5, turnIntegral)); // Clamp integral
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
                telemetry.addData("Turn Error", String.format(Locale.US, "%.2f°", Math.toDegrees(headingError)));
                telemetry.addData("Turn Power", String.format(Locale.US, "%.2f", power));
                telemetry.addData("Shooter RPM", shooterController.getShooterRPM());
                telemetry.update();
            }

            sleep(5);
        }
        driveController.stopDrive();

        // Reset integral for next turn
        turnIntegral = 0.0;
    }

    /**
     * Ultra-fast shooting sequence
     */
    private void quickShoot() {
        // Quick RPM check (max 200ms wait)
        ElapsedTime rpmWait = new ElapsedTime();
        while (opModeIsActive() && !shooterReady && rpmWait.milliseconds() < 200) {
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
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}