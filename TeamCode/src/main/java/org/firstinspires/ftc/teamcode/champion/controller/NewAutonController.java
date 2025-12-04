package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewAutonController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final NewTransferController transferController;
    private final UptakeController uptakeController;
    private final NewShooterController shooterController;
    private final NewIntakeController intakeController;
    private final LimelightAlignmentController limelightController;
    private final NewAutoShootController autoShootController;
    private final NewRampController rampController;

    // NEW: Proven PID controllers
    private final TurnPIDController turnPID;
    private final MovementPIDController movementPID;

    // ========== MOVEMENT PARAMETERS ==========
    @Config
    public static class MovementParams {
        public static double MAX_SPEED = 0.8;
        public static double TOLERANCE_INCHES = 2.0;
        public static double TIMEOUT_MS = 5000;
        public static double HEADING_CORRECTION_KP = 0.15;
    }

    // ========== TURN PARAMETERS ==========
    @Config
    public static class TurnParams {
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
    }

    private Thread pidThread;
    private volatile boolean runPid = false;
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for immediate RPM monitoring
    private Thread rpmMonitorThread;
    private volatile boolean monitorActive = false;
    private volatile boolean isShooting = false;
    private volatile double currentAngle = 0;
    private volatile double baseRampAngle = 121.0;
    private volatile int compensationCount = 0;
    private volatile int recoveryCount = 0;

    public NewAutonController(
            LinearOpMode opMode,
            SixWheelDriveController driveController,
            NewTransferController transferController,
            UptakeController uptakeController,
            NewShooterController shooterController,
            NewIntakeController intakeController,
            LimelightAlignmentController limelightController,
            NewAutoShootController autoShootController,
            NewRampController rampController) {

        this.opMode = opMode;
        this.driveController = driveController;
        this.transferController = transferController;
        this.uptakeController = uptakeController;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.limelightController = limelightController;
        this.autoShootController = autoShootController;
        this.rampController = rampController;

        // Initialize proven PID controllers
        // Get and initialize IMU (Control Hub's built-in IMU used by Pinpoint)
        IMU imu = initializeIMU(opMode);
        this.turnPID = new TurnPIDController(imu);
        this.movementPID = new MovementPIDController();
    }

    private IMU initializeIMU(LinearOpMode opMode) {
        IMU imu = opMode.hardwareMap.get(IMU.class, "imu");

        // Configure IMU orientation based on Control Hub mounting
        // ADJUST THESE VALUES based on your robot's Control Hub orientation!
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,      // Logo facing direction
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD   // USB port facing direction
                )
        );

        imu.initialize(parameters);

        return imu;
    }

    // ========== THREAD MANAGEMENT ==========

    public void startPidUpdateThread() {
        if (pidThread != null && pidThread.isAlive()) return;

        runPid = true;
        pidThread = new Thread(() -> {
            while (runPid && opMode.opModeIsActive()) {
                shooterController.update();
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
                if (isShooting && RPMCompensationForAuton.ENABLE_COMPENSATION && rampController != null) {
                    double currentRPM = shooterController.getRPM();
                    double targetRPM = shooterController.getTargetRPM();
                    double rpmDrop = targetRPM - currentRPM;

                    double targetAngle;
                    if (rpmDrop > RPMCompensationForAuton.RPM_DROP_THRESHOLD) {
                        double angleDecrease = (rpmDrop / 100.0) * RPMCompensationForAuton.ANGLE_COMPENSATION_PER_100RPM;
                        angleDecrease = Math.min(angleDecrease, RPMCompensationForAuton.MAX_ANGLE_COMPENSATION);
                        targetAngle = baseRampAngle - angleDecrease;
                    } else {
                        targetAngle = baseRampAngle;
                    }

                    if (Math.abs(targetAngle - lastAngleSet) > 0.3) {
                        rampController.setTargetAngle(targetAngle);

                        if (targetAngle < lastAngleSet) {
                            compensationCount++;
                        } else if (targetAngle > lastAngleSet) {
                            recoveryCount++;
                        }

                        currentAngle = targetAngle;
                        lastAngleSet = targetAngle;
                    }
                }

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

    // ========== MOVEMENT WITH PROVEN PID ==========

    /**
     * Move robot using proven MovementPIDController
     */
    public void moveRobot(double distanceInches, double maxSpeed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startHeading = driveController.getHeading();

        // Set target distance
        movementPID.setTarget(Math.abs(distanceInches));
        double direction = Math.signum(distanceInches);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            // Calculate distance traveled
            double currentDistance = Math.abs(driveController.getX() - startX);

            // Check if finished
            if (movementPID.isFinished(currentDistance, MovementParams.TOLERANCE_INCHES)) {
                break;
            }

            // Check timeout
            if (timer.milliseconds() > MovementParams.TIMEOUT_MS) {
                break;
            }

            // Get PID output
            double pidOutput = movementPID.update(currentDistance);
            double speed = pidOutput * direction;

            // Apply max speed limit
            speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

            // Heading correction
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = MovementParams.HEADING_CORRECTION_KP * headingError;

            // Apply to motors
            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    // ========== TURN WITH PROVEN PID ==========

    /**
     * Turn to heading using proven TurnPIDController
     */
    public void turnToHeading(double targetDegrees) {
        // Set target
        turnPID.setTarget(targetDegrees);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opMode.opModeIsActive()) {
            driveController.updateOdometry();

            // Check if finished
            if (turnPID.isFinished(TurnParams.TOLERANCE_DEG)) {
                break;
            }

            // Check timeout
            if (timer.milliseconds() > TurnParams.TIMEOUT_MS) {
                break;
            }

            // Get PID output
            double power = turnPID.update();

            // Apply to motors (tank turn)
            driveController.tankDrive(-power, power);

            opMode.sleep(10);
        }

        driveController.stopDrive();
    }

    public double getCurrentHeading() {
        return turnPID.getCurrentHeading();
    }

    /**
     * Quick shoot with IMMEDIATE RPM compensation
     */
    public void quickShoot() {
        compensationCount = 0;
        recoveryCount = 0;

        baseRampAngle = (rampController != null) ? rampController.getTargetAngle() : 121.0;
        currentAngle = baseRampAngle;

        // Wait for RPM stabilization
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 300) {
            if (Math.abs(shooterController.getRPM() - shooterController.getTargetRPM()) < 150) {
                break;
            }
            opMode.sleep(20);
        }

        // Optional alignment
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

        // Activate RPM monitoring
        isShooting = true;

        // Start shooting
        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        intakeController.setState(true);
        intakeController.update();

        timer.reset();

        // Monitor during shooting
        while (opMode.opModeIsActive() && timer.milliseconds() < SHOOT_DURATION) {
            transferController.update();
            uptakeController.update();
            intakeController.update();

            double currentRPM = shooterController.getRPM();
            double targetRPM = shooterController.getTargetRPM();

            opMode.telemetry.addLine("🔥 SHOOTING");
            opMode.telemetry.addData("RPM", "%.0f / %.0f", currentRPM, targetRPM);
            opMode.telemetry.addData("Angle", "%.1f°", currentAngle);
            opMode.telemetry.addData("Compensations", compensationCount);
            opMode.telemetry.update();

            opMode.sleep(50);
        }

        isShooting = false;

        // Stop shooting
        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        intakeController.setState(false);
        intakeController.update();
    }

    public void warmupShooter() {
        timer.reset();
        while (timer.milliseconds() < 500) {
            if (Math.abs(shooterController.getRPM() - CONSTANT_SHOOTER_RPM) < 150) {
                break;
            }
            opMode.sleep(50);
        }
    }

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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}