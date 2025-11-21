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

    // ========== MOVEMENT PID PARAMETERS ==========
    @Config
    public static class MovementPID {
        public static double kP = 0.03;
        public static double kI = 0.0;
        public static double kD = 0.007;
        public static double MIN_SPEED = 0.1;
        public static double MAX_SPEED = 0.8;
        public static double TOLERANCE = 1.0;
        public static double TIMEOUT_MS = 5000;
        public static double HEADING_CORRECTION_KP = 0.1;
        public static double DECEL_DISTANCE = 9.2;
    }

    // ========== TURN PID PARAMETERS ==========
    @Config
    public static class TurnPID {
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.04;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
        public static double TOLERANCE_DEG = 2.0;
        public static double TIMEOUT_MS = 3000;
    }

    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static long SHOOT_DURATION = 1400;

    private Thread pidThread;
    private volatile boolean runPid = false;
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

            // Heading correction
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = MovementPID.HEADING_CORRECTION_KP * headingError;

            // Check completion
            if (Math.abs(error) < MovementPID.TOLERANCE) {
                break;
            }

            // Check timeout
            if (timer.milliseconds() > MovementPID.TIMEOUT_MS) {
                break;
            }

            // Get base speed from PID
            double pidOutput = movePID.getOutput(timer.seconds(), -error);
            double speed = pidOutput;

            // DECELERATION ZONE - gradually reduce max speed near target
            double effectiveMaxSpeed = maxSpeed;
            if (Math.abs(error) < MovementPID.DECEL_DISTANCE) {
                double decelFactor = Math.abs(error) / MovementPID.DECEL_DISTANCE;
                decelFactor = decelFactor * decelFactor; // Quadratic for smooth curve
                effectiveMaxSpeed = MovementPID.MIN_SPEED +
                        (maxSpeed - MovementPID.MIN_SPEED) * decelFactor;
            }

            // Apply MIN_SPEED only when far from target
            if (Math.abs(speed) < MovementPID.MIN_SPEED && Math.abs(error) > MovementPID.TOLERANCE * 2) {
                speed = Math.signum(speed) * MovementPID.MIN_SPEED;
            }

            // Clamp to effective max speed
            speed = Math.max(-effectiveMaxSpeed, Math.min(effectiveMaxSpeed, speed));
            speed *= direction;

            // Apply heading correction
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

            double pidOutput = turnPID.getOutput(timer.seconds(), -headingError);

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

    // ========== SHOOTING ==========

    public void quickShoot() {
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 300) {
            if (Math.abs(shooterController.getShooterRPM() - shooterController.getTargetRPM()) < 150) {
                break;
            }
            opMode.sleep(20);
        }

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

        transferController.transferFull();
        intakeController.intakeFull();

        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < SHOOT_DURATION) {
            opMode.sleep(50);
        }

        transferController.transferStop();
        intakeController.intakeStop();
    }

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