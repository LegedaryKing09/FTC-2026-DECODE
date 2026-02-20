package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton;
import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;
import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutonController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;

import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class AutonMethods {
    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final NewTransferController transferController;
    private final UptakeController uptakeController;
    private final NewShooterController shooterController;
    private final NewIntakeController intakeController;
    private final NewRampController rampController;
    private final NewAutonController autonController;
    private final AutoTankDrive tankDrive;
    private final TurretController turret;

    public org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    public AutonMethods(
            LinearOpMode opMode,
            SixWheelDriveController driveController,
            NewTransferController transferController,
            UptakeController uptakeController,
            NewShooterController shooterController,
            NewIntakeController intakeController,
            NewRampController rampController,
            NewAutonController autonController,
            AutoTankDrive tankDrive,
            TurretController turret) {

        this.opMode = opMode;
        this.driveController = driveController;
        this.transferController = transferController;
        this.uptakeController = uptakeController;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.rampController = rampController;
        this.autonController = autonController;
        this.tankDrive = tankDrive;
        this.turret = turret;
    }
    public AnalogInput uptakeSwitch;

    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;
    public static double AUTON_START_X = 18.5;
    public static double AUTON_START_Y = 16;
    public static double AUTON_START_HEADING = -135.0;

    // turning perfection
    public static double HEADING_CORRECTION_KP = 0.005;
    public static double HEADING_CORRECTION_MAX_VEL = 0.16;
    public static int HEADING_STABLE_SAMPLES = 3;
    public static double HEADING_TIMEOUT_MS = 280;

    // Timing parameters
    public static long INTAKE_TIME_MS = 280;
    public static long SHOOT_TIME_MS = 3000;
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    // turret angles
    public static double AUTO_AIM_ANGLE = 180.0;

    public void shootBalls() {
        // Aim turret at shoot target using current heading
        if (turret != null) {
            double heading = Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble())
                    + AUTON_START_HEADING;
            turret.setFieldAngle(AUTO_AIM_ANGLE);
            turret.enableAutoAim();
            turret.updateAutoAim(heading);
        }

        // Wait for RPM stabilization
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 200) {
            if (Math.abs(shooterController.getRPM() - shooterController.getTargetRPM()) < 150) {
                break;
            }
            sleep(20);
        }

        // Start ALL systems for shooting
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        timer.reset();
        int ballsShotCount = 0;
        boolean lastBallState = false; // sensor reading from previous state - for counting exactly one for one ball

        while (opMode.opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            // Get current values
            boolean ballDetected = isBallAtUptake();

            // Count balls shot
            if (ballDetected && !lastBallState) {
                ballsShotCount++;
                if (ballsShotCount >= 3) {
                    sleep(200);  // Let last ball clear
                    break;
                }
            }
            lastBallState = ballDetected;

            // Update all controllers to keep them running
            intakeController.update();
            transferController.update();
            uptakeController.update();

            sleep(30);
        }

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        sleep(200);
    }


    public void intakeSpline(double splineX, double splineY, double splineAngle) {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .splineTo(new Vector2d(splineX, splineY), Math.toRadians(splineAngle))
                .build();

        // Create a custom action that combines RoadRunner movement with intake control
        Action intakeAction = new Action() {
            private final Action moveAction = moveForward;

            @Override
            public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                checkUptakeSwitch();
                intakeController.update();
                transferController.update();
                uptakeController.update();
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
            uptakeController.update();
            sleep(30);
        }

        // Stop all systems
        intakeModeActive = false;
        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();
    }

    public void intakeXForward(double distance) {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + distance)
                .build();

        // Create a custom action that combines RoadRunner movement with intake control
        Action intakeAction = new Action() {
            private final Action moveAction = moveForward;

            @Override
            public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                checkUptakeSwitch();
                intakeController.update();
                transferController.update();
                uptakeController.update();
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
            uptakeController.update();
            sleep(30);
        }

        // Stop all systems
        intakeModeActive = false;
        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();
    }

    public void intakeYForward(double distance) {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y + distance)
                .build();

        // Create a custom action that combines RoadRunner movement with intake control
        Action intakeAction = new Action() {
            private final Action moveAction = moveForward;

            @Override
            public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                checkUptakeSwitch();
                intakeController.update();
                transferController.update();
                uptakeController.update();
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
            uptakeController.update();
            sleep(30);
        }

        // Stop all systems
        intakeModeActive = false;
        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();
    }

    public void BackwardTurret(double distance) {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action moveBackward = tankDrive.actionBuilder(currentPose)
                .lineToY(distance)
                .build();
        Actions.runBlocking(moveBackward);
    }

    public boolean isBallAtUptake() {
        if (uptakeSwitch == null) return true;
        return uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
    }

    public void startShooterThread() {
        runShooter = true;
        shooterThread = new Thread(() -> {
            while (runShooter && opMode.opModeIsActive()) {
                shooterController.update();
                if (rampController != null) {
                    rampController.update();
                }
                sleep(20);
            }
        });
        shooterThread.setPriority(Thread.MAX_PRIORITY);
        shooterThread.start();
    }

    public void cleanup() {
        double fieldX = AUTON_START_X;
        double fieldY = AUTON_START_Y;
        double fieldHeading = Math.toRadians(AUTON_START_HEADING);
        double rawX, rawY;
        double deltaX, deltaY;
        double deltaHeading;

        try {
            // Try to get pose from RoadRunner's pinpoint localizer
            Pose2d rawPose = tankDrive.pinpointLocalizer.getPose();
            rawX = rawPose.position.x;
            rawY = rawPose.position.y;
            deltaHeading = rawPose.heading.toDouble();  // This is delta from start (0)

            // === COORDINATE TRANSFORMATION ===
            // SWAP_XY = true, NEGATE_X = false, NEGATE_Y = true
            deltaX = rawY;       // Swapped, not negated
            deltaY = -rawX;      // Swapped, then negated

            // Add delta to starting position
            fieldX = AUTON_START_X + deltaX;
            fieldY = AUTON_START_Y + deltaY;
            fieldHeading = Math.toRadians(AUTON_START_HEADING) + deltaHeading;  // Add starting heading

        } catch (Exception e) {
            // If RoadRunner fails, try the SixWheelDriveController odometry
            telemetry.addData("WARNING", "RoadRunner getPose failed: " + e.getMessage());
            try {
                if (driveController != null) {
                    driveController.updateOdometry();
                    rawX = driveController.getX();
                    rawY = driveController.getY();
                    deltaHeading = driveController.getHeading();

                    // Same transformation
                    deltaX = rawY;
                    deltaY = -rawX;
                    fieldX = AUTON_START_X + deltaX;
                    fieldY = AUTON_START_Y + deltaY;
                    fieldHeading = Math.toRadians(AUTON_START_HEADING) + deltaHeading;
                }
            } catch (Exception e2) {
                telemetry.addData("WARNING", "DriveController also failed: " + e2.getMessage());
                // Use starting position as fallback
            }
        }

        // Save to PoseStorage (even if we only have starting position)
        try {
            PoseStorage.currentPose = new Pose2d(
                    new Vector2d(fieldX, fieldY),
                    fieldHeading
            );
        } catch (Exception e) {
            telemetry.addData("WARNING", "Pose2d creation failed: " + e.getMessage());
        }

        // Save turret angle
        if (turret != null) {
            try {
                PoseStorage.turretAngle = turret.getTurretAngle();
            } catch (Exception e) {
                PoseStorage.turretAngle = 0;
            }
        }
        telemetry.addData("Field Pose SAVED", "x=%.1f, y=%.1f, h=%.1f°",
                fieldX, fieldY, Math.toDegrees(fieldHeading));
        telemetry.addData("Turret SAVED", "%.1f°", PoseStorage.turretAngle);
        telemetry.update();
        sleep(2000);

        if (autonController != null) {
            autonController.stopPidUpdateThread();
        }

        runShooter = false;
        if (shooterThread != null) {
            try {
                shooterThread.interrupt();
                shooterThread.join(500);
            } catch (Exception e) {
                // Thread cleanup failed - thread may have already stopped
            }
        }

        shooterController.stopShooting();
        intakeController.setState(false);
        intakeController.update();
        transferController.setState(false);
        transferController.update();
        uptakeController.setState(false);
        uptakeController.update();

    }

    public void checkUptakeSwitch() {
        if (uptakeSwitch == null || uptakeController == null || !intakeModeActive) {
            return;
        }

        boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

        if (ballDetected && !uptakeStoppedBySwitch) {
            uptakeController.setState(false);
            uptakeController.update();
            uptakeStoppedBySwitch = true;
        } else if (!ballDetected && uptakeStoppedBySwitch) {
            uptakeController.setState(true);
            uptakeController.update();
            uptakeStoppedBySwitch = false;
        }
    }

    public void HeadingCorrection(double targetAngleDegrees, double toleranceDegrees) {
        final double kP = HEADING_CORRECTION_KP;
        final double maxAngularVel = HEADING_CORRECTION_MAX_VEL;
        final int stableSamplesRequired = HEADING_STABLE_SAMPLES;
        final double timeoutMs = HEADING_TIMEOUT_MS;

        final int maxAttempts = 15;
        int attemptCount = 0;
        int stableCount = 0;

        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opMode.opModeIsActive()
                && attemptCount < maxAttempts
                && timeout.milliseconds() < timeoutMs) {

            tankDrive.updatePoseEstimate();
            Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
            double currentAngleDeg = Math.toDegrees(currentPose.heading.toDouble());

            // normalize error
            double headingError = targetAngleDegrees - currentAngleDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            if (Math.abs(headingError) <= toleranceDegrees) {
                stableCount++;
                if (stableCount >= stableSamplesRequired) break;
            } else {
                stableCount = 0;
            }

            double angularVel = kP * headingError;
            angularVel = Math.max(-maxAngularVel, Math.min(maxAngularVel, angularVel));

            tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), angularVel));

            sleep(30);
            attemptCount++;
        }

        tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(50);
    }

    /**
     * Enable turret auto-aim at AUTO_AIM_ANGLE.
     * Call once — turret will compensate for heading in movement method loops.
     */
    public void autoAimTurretLeft() {
        if (turret == null) return;
        turret.setFieldAngle(AUTO_AIM_ANGLE);
        turret.enableAutoAim();
    }


}