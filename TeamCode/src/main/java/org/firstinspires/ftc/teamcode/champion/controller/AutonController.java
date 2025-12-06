package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import fr.charleslabs.simplypid.SimplyPID;

@Config
public class AutonController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final TransferController transferController;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final LimelightAlignmentController limelightController;
    private final AutoShootController autoShootController;
    private RampController rampController;

    // ========== MOVEMENT PID PARAMETERS ==========
    @Config
    public static class MovementPID {
        public static double kP = 0.034;
        public static double kI = 0.0;
        public static double kD = 0.0093;
        public static double MIN_SPEED = 0.1;
        public static double MAX_SPEED = 0.8;
        public static double TOLERANCE = 2.0;
        public static double TIMEOUT_MS = 5000;
        public static double HEADING_CORRECTION_KP = 0.15;
        public static double DECEL_DISTANCE = 16.0;  // INCREASED from 9.5
    }

    // ========== TURN PID PARAMETERS ==========
    @Config
    public static class TurnPID {
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.03;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
        public static double TOLERANCE_DEG = 2.0;
        public static double TIMEOUT_MS = 3000;
    }

    // Configuration
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static long SHOOT_DURATION = 1400;

    // RPM Compensation Parameters
    @Config
    public static class RPMCompensationForAuton {
        public static boolean ENABLE_COMPENSATION = true;
        public static double RPM_DROP_THRESHOLD = 100.0;
        public static double ANGLE_COMPENSATION_PER_100RPM = 10.0;
        public static double MAX_ANGLE_COMPENSATION = 50.0;
        public static long RPM_CHECK_INTERVAL = 30;
    }

    private Thread pidThread;
    private volatile boolean runPid = false;
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for immediate RPM monitoring (same as ShootingTest)
    private Thread rpmMonitorThread;
    private volatile boolean monitorActive = false;
    private volatile boolean isShooting = false;  // Flag to enable/disable compensation
    private volatile double currentAngle = 0;
    private volatile double baseRampAngle = 121.0;
    private volatile int compensationCount = 0;
    private volatile int recoveryCount = 0;

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
        this.rampController = null;
    }

    public void setRampController(RampController rampController) {
        this.rampController = rampController;
    }

    // ========== THREAD MANAGEMENT ==========

    public void startPidUpdateThread() {
        if (pidThread != null && pidThread.isAlive()) return;

        runPid = true;
        pidThread = new Thread(() -> {
            while (runPid && opMode.opModeIsActive()) {
                shooterController.updatePID();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        pidThread.setPriority(Thread.MAX_PRIORITY);
        pidThread.start();

        startImmediateRPMMonitor();

    }

    public void stopPidUpdateThread() {
        runPid = false;

        monitorActive = false;

        if (pidThread != null) {
            try {
                pidThread.interrupt();
            } catch (Exception e) {
                // Ignore
            }
        }
    }

    private void startImmediateRPMMonitor() {
        monitorActive = true;
        rpmMonitorThread = new Thread(() -> {
            double lastAngleSet = baseRampAngle;

            while (opMode.opModeIsActive() && monitorActive) {
                // Only compensate while actively shooting
                if (isShooting && RPMCompensationForAuton.ENABLE_COMPENSATION && rampController != null) {
                    double currentRPM = shooterController.getShooterRPM();
                    double targetRPM = shooterController.getTargetRPM();
                    double rpmDrop = targetRPM - currentRPM;

                    // Calculate target angle based on current RPM
                    double targetAngle;
                    if (rpmDrop > RPMCompensationForAuton.RPM_DROP_THRESHOLD) {
                        // RPM is LOW - decrease angle to compensate
                        double angleDecrease = (rpmDrop / 100.0) * RPMCompensationForAuton.ANGLE_COMPENSATION_PER_100RPM;
                        angleDecrease = Math.min(angleDecrease, RPMCompensationForAuton.MAX_ANGLE_COMPENSATION);
                        targetAngle = baseRampAngle - angleDecrease;
                    } else {
                        // RPM is GOOD - return to base angle
                        targetAngle = baseRampAngle;
                    }

                    // IMMEDIATE adjustment - no delay!
                    // Only update if angle changed significantly to avoid servo jitter
                    if (Math.abs(targetAngle - lastAngleSet) > 0.3) {
                        rampController.setAngle(targetAngle);

                        // Track compensation vs recovery
                        if (targetAngle < lastAngleSet) {
                            compensationCount++;
                        } else if (targetAngle > lastAngleSet) {
                            recoveryCount++;
                        }

                        currentAngle = targetAngle;
                        lastAngleSet = targetAngle;
                    }
                }

                // Run as fast as possible - minimal sleep
                // This ensures we catch RPM drops within ~5ms
                try {
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        rpmMonitorThread.setPriority(Thread.MAX_PRIORITY - 1);
        rpmMonitorThread.start();
    }

    // ========== MOVEMENT WITH DECELERATION ==========

    public void moveRobot(double distanceInches, double maxSpeed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startHeading = driveController.getHeading();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        SimplyPID movePID = new SimplyPID(
                0.0,
                MovementPID.kP,
                MovementPID.kI,
                MovementPID.kD
        );
        movePID.setOuputLimits(-MovementPID.MAX_SPEED, MovementPID.MAX_SPEED);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double error = targetDistance - currentDistance;

            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = MovementPID.HEADING_CORRECTION_KP * headingError;

            if (Math.abs(error) < MovementPID.TOLERANCE) {
                break;
            }

            if (timer.milliseconds() > MovementPID.TIMEOUT_MS) {
                break;
            }

            // FIXED: Removed negative sign from error
            double pidOutput = movePID.getOutput(timer.seconds(), error);
            double speed = pidOutput;

            double effectiveMaxSpeed = maxSpeed;
            if (Math.abs(error) < MovementPID.DECEL_DISTANCE) {
                double decelFactor = Math.abs(error) / MovementPID.DECEL_DISTANCE;
                decelFactor = decelFactor * decelFactor;
                effectiveMaxSpeed = MovementPID.MIN_SPEED +
                        (maxSpeed - MovementPID.MIN_SPEED) * decelFactor;
            }

            if (Math.abs(speed) < MovementPID.MIN_SPEED && Math.abs(error) > MovementPID.TOLERANCE * 2) {
                speed = Math.signum(speed) * MovementPID.MIN_SPEED;
            }

            speed = Math.max(-effectiveMaxSpeed, Math.min(effectiveMaxSpeed, speed));
            speed *= direction;

            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    // ========== TURN TO HEADING ==========

    public void turnToHeading(double targetDegrees) {
        double targetRad = Math.toRadians(targetDegrees);
        double toleranceRad = Math.toRadians(TurnPID.TOLERANCE_DEG);

        SimplyPID turnPID = new SimplyPID(
                0.0,
                TurnPID.kP,
                TurnPID.kI,
                TurnPID.kD
        );
        turnPID.setOuputLimits(-TurnPID.MAX_POWER, TurnPID.MAX_POWER);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(targetRad - currentHeading);

            if (Math.abs(headingError) < toleranceRad) {
                break;
            }

            if (timer.milliseconds() > TurnPID.TIMEOUT_MS) {
                break;
            }

            // FIXED: Removed negative sign from error
            double pidOutput = turnPID.getOutput(timer.seconds(), headingError);

            double power = pidOutput;
            if (Math.abs(power) < TurnPID.MIN_POWER && Math.abs(headingError) > toleranceRad) {
                power = Math.signum(power) * TurnPID.MIN_POWER;
            }

            driveController.tankDrive(-power, power);

            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    public double getCurrentHeading() {
        driveController.updateOdometry();
        return Math.toDegrees(driveController.getHeading());
    }


    public void quickShoot() {
        // Reset compensation counters
        compensationCount = 0;
        recoveryCount = 0;

        // Set base angle for compensation calculations
        baseRampAngle = (rampController != null) ? rampController.getAngle() : 121.0;
        currentAngle = baseRampAngle;

        // Wait for RPM to stabilize (max 300ms)
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 300) {
            if (Math.abs(shooterController.getShooterRPM() - shooterController.getTargetRPM()) < 150) {
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

        // This activates the background monitor thread!
        isShooting = true;

        // Start shooting
        transferController.transferFull();
        intakeController.intakeFull();

        timer.reset();

        // RPM compensation now happens in background thread automatically
        // This loop just handles timing and telemetry
        while (opMode.opModeIsActive() && timer.milliseconds() < SHOOT_DURATION) {
            // Optional: Display real-time telemetry
            double currentRPM = shooterController.getShooterRPM();
            double targetRPM = shooterController.getTargetRPM();

            opMode.telemetry.addLine("🔥 SHOOTING - IMMEDIATE compensation active");
            opMode.telemetry.addData("RPM", "%.0f / %.0f", currentRPM, targetRPM);
            opMode.telemetry.addData("Angle", "%.1f°", currentAngle);
            opMode.telemetry.addData("Compensations", compensationCount);
            opMode.telemetry.addData("Recoveries", recoveryCount);
            opMode.telemetry.addData("Response Time", "~5ms");
            opMode.telemetry.update();

            opMode.sleep(50);  // Telemetry update rate (doesn't affect compensation speed)
        }

        isShooting = false;

        // Stop shooting
        transferController.transferStop();
        intakeController.intakeStop();

        // Log final stats
        if (compensationCount > 0 || recoveryCount > 0) {
            opMode.telemetry.addLine("✓ Shooting complete (IMMEDIATE mode)");
            opMode.telemetry.addData("Compensations applied", compensationCount);
            opMode.telemetry.addData("Recoveries applied", recoveryCount);
            opMode.telemetry.addData("Final angle", "%.1f°", currentAngle);
            opMode.telemetry.update();
        }
    }
    // ====================================================================================

    public void warmupShooter() {
        timer.reset();
        while (timer.milliseconds() < 500) {
            if (Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) < 150) {
                break;
            }
            opMode.sleep(50);
        }
    }

    // ========== PATTERN DETECTION ==========

    public int detectPattern() {
        if (autoShootController == null) return 0;

        try {
            opMode.sleep(300);
            int tagId = autoShootController.getVisibleAprilTagId();

            if (tagId == 21) return 2;
            if (tagId == 22) return 1;
            if (tagId == 23) return 0;
        } catch (Exception e) {
            // Use default
        }
        return 0;
    }

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

    // ========== INTAKE CONTROL ==========

    public void startIntakeThread() {
        // Empty
    }

    public void stopIntakeThread() {
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
}