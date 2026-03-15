package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton;
import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
public class AutonMethodsClose {
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

    public AutonMethodsClose(
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
    public static long BALL_GAP_MS = 2000;  // Gap allowed between consecutive balls at uptake
    public static long SHOOT_DURATION_MS = 5000;  // Total shooting duration
    public static double FAR_RPM = 4100;
    public static double FAR_RAMP = 0.34;
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    // turret angles
    public static double AUTO_AIM_ANGLE = 180.0;

    // shooting target field coordinates (used for both turret aim AND distance-based RPM/ramp)
    public static double SHOOT_TARGET_X = 10.0;
    public static double SHOOT_TARGET_Y = 10.0;

    // When true, turret calculates field angle to target once from position,
    // then uses heading-only compensation (for far auton).
    // When false, uses full position tracking every shot (for close auton).
    public static boolean useHeadingOnlyAim = false;

    /**
     * Converts raw Pinpoint pose to field coordinates, accounting for starting heading.
     * Raw Pinpoint gives robot-relative displacement (forward/sideways from start).
     * We rotate by AUTON_START_HEADING to get field-frame displacement.
     *
     * Returns double[3]: {fieldX, fieldY, fieldHeadingDeg}
     */
    public double[] getFieldPosition() {
        double fieldX = AUTON_START_X;
        double fieldY = AUTON_START_Y;
        double fieldHeading = AUTON_START_HEADING;
        try {
            Pose2d rawPose = tankDrive.pinpointLocalizer.getPose();
            double rawX = rawPose.position.x;  // sideways in robot frame
            double rawY = rawPose.position.y;  // forward in robot frame

            // Rotate raw displacement by starting heading to get field displacement
            double startRad = Math.toRadians(AUTON_START_HEADING);
            double cos = Math.cos(startRad);
            double sin = Math.sin(startRad);

            // rawY = forward, rawX = right in robot frame
            // Field X += forward * sin(heading) + right * cos(heading)
            // Field Y += forward * cos(heading) - right * sin(heading)
            // But with SWAP_XY, Pinpoint rawX = sideways, rawY = forward
            fieldX = AUTON_START_X + rawY * sin + rawX * cos;
            fieldY = AUTON_START_Y + rawY * cos - rawX * sin;

            // Heading: CW is negative from Pinpoint, negate to match teleop convention
            fieldHeading = AUTON_START_HEADING - Math.toDegrees(rawPose.heading.toDouble());
        } catch (Exception e) { /* use start defaults */ }
        return new double[]{fieldX, fieldY, fieldHeading};
    }

    /**
     * Hardcoded RPM and ramp for close shooting. Set these from your auton file.
     */
    public static double CLOSE_RPM = 3400.0;
    public static double CLOSE_RAMP = 0.0;

    /**
     * Runs ALL systems at full power until balls are gone.
     * NO RPM gating, NO turret auto-aiming, NO distance calculation.
     * Uses hardcoded CLOSE_RPM and CLOSE_RAMP.
     * Exits when no ball for BALL_GAP_MS, or after SHOOT_DURATION_MS safety timeout.
     */
    public void shootBalls() {
        // Set hardcoded RPM and ramp
        shooterController.setTargetRPM(CLOSE_RPM);
        if (rampController != null) rampController.setTargetAngle(CLOSE_RAMP);

        // Wait for RPM stabilization (brief)
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 200) {
            if (Math.abs(shooterController.getRPM() - shooterController.getTargetRPM()) < 150) {
                break;
            }
            sleep(20);
        }

        // Start ALL systems at full power — no RPM gating
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Shoot until no balls remain
        ElapsedTime gapTimer = new ElapsedTime();
        ElapsedTime shootTimer = new ElapsedTime();
        gapTimer.reset();
        shootTimer.reset();

        while (opMode.opModeIsActive() && shootTimer.milliseconds() < SHOOT_DURATION_MS) {
            boolean ballsInBot = (uptakeSwitch != null && uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD);

            if (ballsInBot) {
                gapTimer.reset();
            } else if (gapTimer.milliseconds() >= BALL_GAP_MS) {
                break;
            }

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

    /**
     * Shows current field position on telemetry. Call after any move.
     */
    public void showPosition(String label) {
        double[] pos = getFieldPosition();
        telemetry.addData("=== " + label + " ===", "");
        telemetry.addData("Field X", "%.1f", pos[0]);
        telemetry.addData("Field Y", "%.1f", pos[1]);
        telemetry.addData("Heading", "%.1f°", pos[2]);
        telemetry.update();
    }

    /**
     * Calculates the turret servo position needed to aim at SHOOT_TARGET from
     * the current robot position and heading.
     *
     * Uses the same math as TurretController:
     *   fieldAngle = atan2(-dx, -dy)   (0° = facing -Y = forward)
     *   turretAngle = fieldAngle - (-heading)  (INVERT_HEADING)
     *   servoPos = 0.5 + (-turretAngle / 315.0)
     */
    public double calculateTurretServoPosition() {
        double[] pos = getFieldPosition();
        double fieldX = pos[0];
        double fieldY = pos[1];
        double heading = pos[2];

        double dx = SHOOT_TARGET_X - fieldX;
        double dy = SHOOT_TARGET_Y - fieldY;
        double fieldAngle = Math.toDegrees(Math.atan2(-dx, -dy));

        // INVERT_HEADING: turretAngle = fieldAngle - (-heading) = fieldAngle + heading
        double turretAngle = fieldAngle + heading;

        // Normalize to -180..180
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle <= -180) turretAngle += 360;

        double servoPos = 0.5 + (-turretAngle / 315.0);
        return Math.max(0.0, Math.min(1.0, servoPos));
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
                .lineToY(distance)
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
        // Use the corrected rotation-based field position
        double[] pos = getFieldPosition();
        double fieldX = pos[0];
        double fieldY = pos[1];
        double fieldHeading = Math.toRadians(pos[2]);

        // Save to PoseStorage (even if we only have starting position)
        try {
            PoseStorage.currentPose = new Pose2d(new Vector2d(fieldX, fieldY), fieldHeading);
            PoseStorage.savePose(hardwareMap.appContext, fieldX, fieldY, fieldHeading, turret.getTurretAngle());
        } catch (Exception e) {
            telemetry.addData("WARNING", "Pose2d creation failed: " + e.getMessage());
        }
        telemetry.addData("Field Pose SAVED", "x=%.1f, y=%.1f, h=%.1f°",
                fieldX, fieldY, Math.toDegrees(fieldHeading));
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
     * Parabolic shooting curve fitted to:
     *   (28in, 3250 RPM, 0.0 ramp), (64in, 3450 RPM, 0.16), (100in, 3800 RPM, 0.3)
     * Returns double[]{rpm, rampAngle}. Callers add RPM_READY when setting shooter target.
     */
    private double[] getShootingParamsForDistance(double distance) {
        double rpm, rampAngle;
        if (distance <= 28.0) {
            rpm = 3250.0;
            rampAngle = 0.0;
        } else if (distance <= 108.0) {
            rpm = (25.0 / 432.0) * distance * distance + (25.0 / 108.0) * distance + (86350.0 / 27.0);
            rampAngle = (-1.0 / 129600.0) * distance * distance + (167.0 / 32400.0) * distance + (-56.0 / 405.0);
        } else {
            rpm = FAR_RPM;
            rampAngle = FAR_RAMP;
        }
        return new double[]{rpm, rampAngle};
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