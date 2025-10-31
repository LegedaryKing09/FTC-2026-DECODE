package org.firstinspires.ftc.teamcode.champion.Auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.PurePursuitController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

@Config
@Autonomous(name = "Basic Auton", group = "Competition")
public class BasicAuton extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;
    PurePursuitController pursuitController;

    // Autonomous parameters - OPTIMIZED FOR SPEED
    public static double CONSTANT_SHOOTER_RPM = 2800.0;  // Fixed RPM for entire autonomous
    public static double CONSTANT_RAMP_ANGLE = 120.0;    // Fixed ramp angle for entire autonomous
    public static double INTAKE_SPEED = 0.2;  // Slightly faster intake
    public static double BACKWARD_DISTANCE_INCHES = 50.0;
    public static double FORWARD_DISTANCE_INCHES = 35;
    public static double REPOSITIONING_DISTANCE = 20;
    public static double MOVEMENT_SPEED = 0.35;  // Slightly faster movement

    // REDUCED TIMEOUTS AND DELAYS FOR SPEED
    public static long ALIGNMENT_TIMEOUT = 600;  // Reduced from 1000ms
    public static long SHOOT_DURATION = 1500;    // Reduced from 1800ms but still enough for 2 balls
    public static long SETTLE_TIME = 200;        // Reduced from 500ms
    public static double TURN_ANGLE_DEGREES = 21.0;
    public static double ALIGNMENT_THRESHOLD = 2.0;  // Increased from 1.0 for faster alignment
    public static long SHOOTER_WARMUP_TIME = 800;    // Reduced from 1500ms
    public static double RPM_TOLERANCE = 250;        // Increased from 200 for faster acceptance

    // PID Update frequency
    public static int PID_UPDATE_INTERVAL_MS = 10;  // More frequent PID updates

    private final ElapsedTime pidUpdateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);
        pursuitController = new PurePursuitController();
        pursuitController.setParameters(4.0, 0.6, 11.0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        LimelightAlignmentController tempLimelight = null;
        try {
            tempLimelight = new LimelightAlignmentController(this, driveController);
            tempLimelight.setTargetTag(AutoShootController.APRILTAG_ID);
            telemetry.addData("Limelight", "✅ Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
        }
        limelightController = tempLimelight;

        // Initialize auto shoot controller (for telemetry only)
        autoShootController = new AutoShootController(
                this,
                driveController,
                shooterController,
                intakeController,
                transferController,
                limelightController,
                rampController
        );

        telemetry.addLine("=== OPTIMIZED AUTON READY ===");
        telemetry.addData("Constant Shooter RPM", CONSTANT_SHOOTER_RPM);
        telemetry.addData("Constant Ramp Angle", CONSTANT_RAMP_ANGLE);
        telemetry.addData("Shoot Duration", SHOOT_DURATION + " ms");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // Reset PID timer
        pidUpdateTimer.reset();

        // Start shooter at constant RPM
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);

        // Set ramp to constant angle
        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        // ===== SHORTER WARMUP WITH CONTINUOUS PID =====
        ElapsedTime warmupTimer = new ElapsedTime();
        while (opModeIsActive() && warmupTimer.milliseconds() < SHOOTER_WARMUP_TIME) {
            // Aggressive PID updates during warmup
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }

            sleep(5); // Shorter sleep for more responsive loop
        }

        // Execute backward movement WITH PID UPDATES
        moveBackwardWithOdometry(BACKWARD_DISTANCE_INCHES);

        // Minimal settle time
        sleep(SETTLE_TIME);

        // Execute FIRST SHOT - FAST VERSION
        executeFastShootSequence();

        // Minimal delay between shots
        sleep(200);

        // Start intake for second set
        intakeController.intakeFull();

        // FAST REPOSITIONING - all movements maintain shooter
        turnToHeadingFast(TURN_ANGLE_DEGREES);
        ForwardForIntakeWithShooterUpdate(FORWARD_DISTANCE_INCHES);
        sleep(200); // Brief intake time
        moveBackwardWithOdometryAndShooterUpdate(REPOSITIONING_DISTANCE);
        turnToHeadingFast(0);

        // Stop intake
        intakeController.intakeStop();
        sleep(100); // Minimal delay

        // Execute SECOND SHOT - FAST VERSION
        executeFastShootSequence();

        // Stop shooter at end
        sleep(300);
        shooterController.shooterStop();
    }

    /**
     * FAST shoot sequence - minimal delays, aggressive acceptance criteria
     */
    @SuppressLint("DefaultLocale")
    private void executeFastShootSequence() {

        // STEP 1: VERY QUICK RPM CHECK (don't wait long)
        double currentRPM = shooterController.getShooterRPM();
        if (Math.abs(currentRPM - CONSTANT_SHOOTER_RPM) > RPM_TOLERANCE) {
            // Only wait maximum 300ms for RPM
            ElapsedTime rpmTimer = new ElapsedTime();
            while (opModeIsActive() &&
                    Math.abs(shooterController.getShooterRPM() - CONSTANT_SHOOTER_RPM) > RPM_TOLERANCE &&
                    rpmTimer.milliseconds() < 300) {
                shooterController.updatePID();
                sleep(10);
            }
        }

        // STEP 2: FAST ALIGNMENT (if Limelight available)
        if (limelightController != null) {
            try {
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                ElapsedTime alignTimer = new ElapsedTime();

                // Quick alignment loop - don't be perfectionist
                while (opModeIsActive() && alignTimer.milliseconds() < ALIGNMENT_TIMEOUT) {
                    // Keep updating shooter during alignment
                    if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                        shooterController.updatePID();
                        pidUpdateTimer.reset();
                    }

                    limelightController.align(AutoShootController.APRILTAG_ID);

                    // Accept "good enough" alignment faster
                    if (limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                        break;
                    }

                    sleep(10);
                }

                limelightController.stopAlignment();
                driveController.stopDrive();

                // Minimal stabilization
                sleep(50);

            } catch (Exception e) {
                // Ignore alignment errors, shoot anyway
            }
        }

        // STEP 3: SHOOT WITHOUT HESITATION
        transferController.transferFull();
        intakeController.intakeFull();

        // Shooting loop with continuous PID updates
        ElapsedTime shotTimer = new ElapsedTime();
        while (opModeIsActive() && shotTimer.milliseconds() < SHOOT_DURATION) {
            // Aggressive PID updates during shot
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }
            sleep(5);
        }

        // Stop intake and transfer
        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * Fast turn with continuous shooter updates
     */
    @SuppressLint("DefaultLocale")
    private void turnToHeadingFast(double targetHeadingDeg) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        final double HEADING_THRESHOLD = Math.toRadians(3.0); // Less precise but faster
        final double TURN_POWER = 0.4; // Faster turns

        while (opModeIsActive()) {
            // Continuous PID updates
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }

            driveController.updateOdometry();
            double currentHeading = driveController.getHeading();
            double headingError = targetHeadingRad - currentHeading;

            // Normalize error
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            if (Math.abs(headingError) < HEADING_THRESHOLD) {
                driveController.stopDrive();
                break;
            }

            double turnPower = Math.signum(headingError) * TURN_POWER;
            driveController.tankDrive(-turnPower, turnPower);

            sleep(10);
        }
    }

    /**
     * Forward movement with aggressive PID updates
     */
    @SuppressLint("DefaultLocale")
    private void ForwardForIntakeWithShooterUpdate(double distanceInches) {
        driveController.updateOdometry();
        double startX = driveController.getX();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(INTAKE_SPEED, INTAKE_SPEED);

        while (opModeIsActive()) {
            // Aggressive PID updates
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(currentX - startX);

            if (distanceMoved >= Math.abs(distanceInches)) {
                break;
            }

            sleep(10);
        }

        driveController.stopDrive();
    }

    /**
     * Backward movement with aggressive PID updates
     */
    @SuppressLint("DefaultLocale")
    private void moveBackwardWithOdometryAndShooterUpdate(double distanceInches) {
        driveController.updateOdometry();
        double startX = driveController.getX();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        while (opModeIsActive()) {
            // Aggressive PID updates
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(startX - currentX);

            if (distanceMoved >= Math.abs(distanceInches)) {
                break;
            }

            sleep(10);
        }

        driveController.stopDrive();
    }

    /**
     * Initial backward movement with PID updates
     */
    @SuppressLint("DefaultLocale")
    private void moveBackwardWithOdometry(double distanceInches) {
        driveController.updateOdometry();
        double startX = driveController.getX();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
        driveController.tankDriveVelocityNormalized(-MOVEMENT_SPEED, -MOVEMENT_SPEED);

        while (opModeIsActive()) {
            // Keep updating PID
            if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                shooterController.updatePID();
                pidUpdateTimer.reset();
            }

            driveController.updateOdometry();
            double currentX = driveController.getX();
            double distanceMoved = Math.abs(startX - currentX);

            if (distanceMoved >= Math.abs(distanceInches)) {
                break;
            }

            sleep(10);
        }

        driveController.stopDrive();
    }
}