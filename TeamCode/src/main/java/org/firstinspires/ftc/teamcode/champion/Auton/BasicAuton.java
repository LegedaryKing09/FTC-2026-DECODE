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
    public static double BACKWARD_DISTANCE_INCHES = 50.0;

    // Base distances for ball pickup and repositioning
    public static double BASE_PICKUP_DISTANCE = 35.0;      // Base forward distance for ball pickup
    public static double BASE_REPOSITIONING_DISTANCE = 20.0; // Base backward distance for repositioning

    public static double MOVEMENT_SPEED = 0.4;  // Slightly faster movement

    // REDUCED TIMEOUTS AND DELAYS FOR SPEED
    public static long ALIGNMENT_TIMEOUT = 600;  // Reduced from 1000ms
    public static long SHOOT_DURATION = 1500;    // Reduced from 1800ms but still enough for 2 balls
    public static long SETTLE_TIME = 200;        // Reduced from 500ms
    public static double TURN_ANGLE_DEGREES = 21.0;
    public static double PRE_SHOOT_TURN_ANGLE = -45.0;  // Base turn angle before first shot
    public static double PPG_TURN_ADJUSTMENT = 0.0;    // Additional turn for PPG (total -45°)
    public static double PGP_TURN_ADJUSTMENT = 15.0;   // Additional turn for PGP (total -60°)
    public static double GPP_TURN_ADJUSTMENT = 30.0;   // Additional turn for GPP (total -75°)
    public static double TURN_BACK_ANGLE = 66.0;       // Base angle to turn back to (compensates for pre-shoot turn)
    public static double ALIGNMENT_THRESHOLD = 2.0;  // Increased from 1.0 for faster alignment
    public static long SHOOTER_WARMUP_TIME = 800;    // Reduced from 1500ms
    public static double RPM_TOLERANCE = 250;        // Increased from 200 for faster acceptance

    // PID Update frequency
    public static int PID_UPDATE_INTERVAL_MS = 10;  // More frequent PID updates

    public enum Pattern {
        PPG, PGP, GPP;
    }

    // Pattern-based additional distances after shooting
    public static double PPG_ADDITIONAL_DISTANCE = 0.0;  // inches
    public static double PGP_ADDITIONAL_DISTANCE = 12.0;   // inches
    public static double GPP_ADDITIONAL_DISTANCE = 24.0;  // inches

    private Pattern currentPattern = Pattern.PPG;  // default

    // AprilTag ID to Pattern mapping
    private static final java.util.Map<Integer, Pattern> APRILTAG_PATTERN_MAP = new java.util.HashMap<>();
    static {
        APRILTAG_PATTERN_MAP.put(21, Pattern.PPG);
        APRILTAG_PATTERN_MAP.put(22, Pattern.PGP);
        APRILTAG_PATTERN_MAP.put(23, Pattern.GPP);
        // Add more mappings as needed
    }

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

        // Turn -45 degrees before shooting for AprilTag scanning position
        turnToHeadingFast(PRE_SHOOT_TURN_ANGLE);

        // Pause for Limelight scanning and determine pattern
        determinePatternFromAprilTag();
        sleep(300); // Pause for reliable Limelight scanning

        // Execute FIRST SHOT - FAST VERSION


        // FAST REPOSITIONING - all movements maintain shooter
        double adjustedTurnBackAngle = TURN_BACK_ANGLE + getTurnBackAdjustmentForPattern(currentPattern);
        turnToHeadingFast(adjustedTurnBackAngle);
        double pickupDistance = getPickupDistanceForPattern(currentPattern);

        double repositioningDistance = getRepositioningDistanceForPattern(currentPattern);

        executeFastShootSequence();

        // Minimal delay between shots
        sleep(200);

        // Start intake for second set
        intakeController.intakeFull();
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
        driveController.tankDriveVelocityNormalized(MOVEMENT_SPEED, MOVEMENT_SPEED);

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

    /**
     * Determine the pattern based on visible AprilTag
     */
    private void determinePatternFromAprilTag() {
        if (limelightController == null) {
            telemetry.addData("Pattern Detection", "Limelight not available - using default PPG");
            telemetry.update();
            return;
        }

        try {
            int aprilTagId = ((AutoShootController) autoShootController).getVisibleAprilTagId();
            Pattern detectedPattern = APRILTAG_PATTERN_MAP.get(aprilTagId);

            if (detectedPattern != null) {
                currentPattern = detectedPattern;
                telemetry.addData("Detected Pattern", currentPattern + " (AprilTag " + aprilTagId + ")");
            } else {
                telemetry.addData("Pattern Detection", "Unknown AprilTag " + aprilTagId + " - using default PPG");
                currentPattern = Pattern.PPG;
            }
            telemetry.update();
            sleep(100); // Brief display time

        } catch (Exception e) {
            telemetry.addData("Pattern Detection Error", e.getMessage());
            telemetry.addData("Using default pattern", "PPG");
            telemetry.update();
            currentPattern = Pattern.PPG;
        }
    }

    /**
     * Move additional distance after shooting based on detected pattern
     */
    private void moveAdditionalDistanceAfterShoot() {
        double additionalDistance = 0;

        switch (currentPattern) {
            case PPG:
                additionalDistance = PPG_ADDITIONAL_DISTANCE;
                break;
            case PGP:
                additionalDistance = PGP_ADDITIONAL_DISTANCE;
                break;
            case GPP:
                additionalDistance = GPP_ADDITIONAL_DISTANCE;
                break;
        }

        telemetry.addData("Moving additional distance", additionalDistance + " inches for pattern " + currentPattern);
        telemetry.update();

        if (additionalDistance > 0) {
            driveController.updateOdometry();
            double startX = driveController.getX();

            driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
            driveController.tankDriveVelocityNormalized(MOVEMENT_SPEED, MOVEMENT_SPEED);

            while (opModeIsActive()) {
                // Keep updating PID during movement
                if (pidUpdateTimer.milliseconds() >= PID_UPDATE_INTERVAL_MS) {
                    shooterController.updatePID();
                    pidUpdateTimer.reset();
                }

                driveController.updateOdometry();
                double currentX = driveController.getX();
                double distanceMoved = Math.abs(currentX - startX);

                if (distanceMoved >= Math.abs(additionalDistance)) {
                    break;
                }

                sleep(10);
            }

            driveController.stopDrive();
        }
    }

    /**
     * Get pickup distance based on pattern (add additional distance)
     */
    private double getPickupDistanceForPattern(Pattern pattern) {
        double additionalDistance = 0;

        switch (pattern) {
            case PPG:
                additionalDistance = PPG_ADDITIONAL_DISTANCE;
                break;
            case PGP:
                additionalDistance = PGP_ADDITIONAL_DISTANCE;
                break;
            case GPP:
                additionalDistance = GPP_ADDITIONAL_DISTANCE;
                break;
        }

        return BASE_PICKUP_DISTANCE + additionalDistance;
    }

    /**
     * Get repositioning distance based on pattern (add additional distance)
     */
    private double getRepositioningDistanceForPattern(Pattern pattern) {
        double additionalDistance = 0;

        switch (pattern) {
            case PPG:
                additionalDistance = PPG_ADDITIONAL_DISTANCE;
                break;
            case PGP:
                additionalDistance = PGP_ADDITIONAL_DISTANCE;
                break;
            case GPP:
                additionalDistance = GPP_ADDITIONAL_DISTANCE;
                break;
        }

        return BASE_REPOSITIONING_DISTANCE + additionalDistance;
    }

    /**
     * Get turn adjustment for pre-shooting turn based on pattern
     */
    private double getTurnAdjustmentForPattern(Pattern pattern) {
        switch (pattern) {
            case PPG:
                return PPG_TURN_ADJUSTMENT;
            case PGP:
                return PGP_TURN_ADJUSTMENT;
            case GPP:
                return GPP_TURN_ADJUSTMENT;
            default:
                return 0.0;
        }
    }

    /**
     * Get turn back adjustment based on pattern (compensates for pre-shoot turn)
     */
    private double getTurnBackAdjustmentForPattern(Pattern pattern) {
        // Return negative of the pre-shoot adjustment to compensate
        return -getTurnAdjustmentForPattern(pattern);
    }
}