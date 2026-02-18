package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.Auton.SimpleLimelightVision;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;

/**
 * Auto-Aim Shooter with Dual Localization Display
 *
 * FIELD SETUP:
 * - 144" x 144" field
 * - Goal at (0, 0) - upper left corner
 * - Robot starts at (48, 144) - bottom of robot touching wall
 * - Heading 0° = facing toward goal (-Y direction)
 *
 * LOCALIZATION:
 * - Shows BOTH odometry and Limelight positions separately
 * - You choose which to use via DPad Up/Down
 *
 * AUTO-AIM:
 * - Calculates angle from robot to goal
 * - Sets that as the field angle on TurretController
 * - TurretController handles auto-aim to maintain aim
 *
 * Controls:
 *   Left Stick = Drive
 *   Right Stick = Turn
 *   A = Toggle intake
 *   B = Toggle transfer
 *   X = Toggle uptake
 *   Y = Toggle turret auto-aim
 *   LB = Reset odometry
 *   RB = Toggle speed
 *   DPad Up = Use VISION
 *   DPad Down = Use ODOMETRY
 */
@Config
@TeleOp(name = "Auto-Aim Shooter", group = "Competition")
public class OdometrySystemShoot extends LinearOpMode {
    public static double angleToGoal= 45;
    // === FIELD CONFIGURATION ===
    public static double GOAL_X = 0.0;
    public static double GOAL_Y = 0.0;
    public static double START_X = 48.0;
    public static double START_Y = 144.0;
    public static double START_HEADING = 0.0;

    // === LIMELIGHT OFFSET ===
    // Distance from robot CENTER to Limelight (positive = Limelight is forward of center)
    // Example: If robot is 18" long and Limelight is 3" forward of center, set to 3.0
    public static double LIMELIGHT_OFFSET_Y = 6.0;  // inches - adjust for your robot

    // === LOCALIZATION SOURCE ===
    public static boolean USE_VISION = false;

    // === SHOOTER ===
    public static double SHOOTER_RPM = 1500.0;

    // === TELEMETRY ===
    public static double TELEMETRY_INTERVAL_MS = 100;

    // Controllers
    private SixWheelDriveController drive;
    private SimpleLimelightVision vision;
    private TurretController turret;
    private NewShooterController shooter;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;

    // Positions from each system
    private double odoX = START_X;
    private double odoY = START_Y;
    private double odoHeading = START_HEADING;

    private double visionX = 0;
    private double visionY = 0;
    private double visionHeading = 0;
    private boolean visionValid = false;

    // Active position
    private double robotX = START_X;
    private double robotY = START_Y;
    private double robotHeading = START_HEADING;

    // Auto-aim
    private boolean autoAimEnabled = true;

    // Button states
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLB = false;
    private boolean lastRB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Timing
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private double avgLoopTimeMs = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeHardware();
        telemetry.update();

        waitForStart();

        // Initialize
        drive.resetOdometry();
        turret.initialize();

        // Start shooter
        shooter.setTargetRPM(SHOOTER_RPM);
        shooter.startShooting();

        // Enable turret auto-aim
        turret.setFieldAngle(angleToGoal);
        turret.enableAutoAim();

        loopTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            avgLoopTimeMs = avgLoopTimeMs * 0.95 + loopTimeMs * 0.05;

            // === UPDATE SENSORS ===
            drive.updateOdometry();
            turret.update();
            shooter.update();
            intake.update();
            transfer.update();
            if (uptake != null) uptake.update();

            // === UPDATE BOTH LOCALIZATION SYSTEMS ===
            updateOdometryPosition();
            updateVisionPosition();

            // === SELECT ACTIVE POSITION ===
            if (USE_VISION && visionValid) {
                robotX = visionX;
                robotY = visionY;
                robotHeading = visionHeading;
            } else {
                robotX = odoX;
                robotY = odoY;
                robotHeading = odoHeading;
            }

            // === AUTO-AIM: Set target for TurretController ===
            if (autoAimEnabled) {
                turret.setFieldAngle(angleToGoal);
                turret.updateAutoAim(robotHeading);
            }

            // === DRIVETRAIN ===
            double driveInput = -gamepad1.left_stick_y;
            double turnInput = gamepad1.right_stick_x;

            double speedMult = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_SPEED_MULTIPLIER :
                    SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
            double turnMult = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_TURN_MULTIPLIER :
                    SixWheelDriveController.SLOW_TURN_MULTIPLIER;

            drive.arcadeDrive(driveInput * speedMult, turnInput * turnMult);

            // === BUTTON CONTROLS ===
            if (gamepad1.a && !lastA) intake.toggle();
            lastA = gamepad1.a;

            if (gamepad1.b && !lastB) transfer.toggle();
            lastB = gamepad1.b;

            if (gamepad1.x && !lastX) {
                if (uptake != null) uptake.toggle();
            }
            lastX = gamepad1.x;

            // Y = Toggle auto-aim
            if (gamepad1.y && !lastY) {
                autoAimEnabled = !autoAimEnabled;
                if (autoAimEnabled) {
                    turret.setFieldAngle(angleToGoal);
                    turret.enableAutoAim();
                } else {
                    turret.disableAutoAim();
                }
            }
            lastY = gamepad1.y;

            if (gamepad1.left_bumper && !lastLB) {
                drive.resetOdometry();
            }
            lastLB = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !lastRB) {
                drive.toggleSpeedMode();
            }
            lastRB = gamepad1.right_bumper;

            if (gamepad1.dpad_up && !lastDpadUp) {
                USE_VISION = true;
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                USE_VISION = false;
            }
            lastDpadDown = gamepad1.dpad_down;

            // === TELEMETRY ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                updateTelemetry();
            }
        }

        // Cleanup
        drive.stopDrive();
        turret.disableAutoAim();
        shooter.stopShooting();
        intake.setState(false);
        intake.update();
        transfer.setState(false);
        transfer.update();
        if (uptake != null) {
            uptake.setState(false);
            uptake.update();
        }
        if (vision != null) vision.shutdown();
    }

    private void initializeHardware() {
        drive = new SixWheelDriveController(this);

        try {
            vision = new SimpleLimelightVision(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("WARNING: Limelight not found");
            vision = null;
        }

        turret = new TurretController(this);

        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Shooter: " + e.getMessage());
        }
        shooter = new NewShooterController(shooterMotor1,shooterMotor2);

        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception ignored) {}
        intake = new NewIntakeController(intakeMotor);

        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception ignored) {}
        transfer = new NewTransferController(transferMotor);

        CRServo uptakeServo1 = hardwareMap.get(CRServo.class, "servo1");
        CRServo uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        uptake = new UptakeController(uptakeServo1, uptakeServo2);
    }

    private void updateOdometryPosition() {
        double rawOdoX = drive.getX();
        double rawOdoY = drive.getY();
        double rawOdoHeading = drive.getHeadingDegrees();

        odoX = START_X + rawOdoY;
        odoY = START_Y - rawOdoX;
        odoHeading = normalizeAngle(START_HEADING + rawOdoHeading);
    }

    private void updateVisionPosition() {
        if (vision == null) {
            visionValid = false;
            return;
        }

        visionValid = vision.hasValidData();
        if (!visionValid) return;

        // Use existing methods from SimpleLimelightVision - already handles offset!
        visionX = vision.getRobotCenterX(LIMELIGHT_OFFSET_Y);
        visionY = vision.getRobotCenterY(LIMELIGHT_OFFSET_Y);
        visionHeading = vision.getHeading();
    }

    private double calculateAngleToGoal() {
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double angle = Math.toDegrees(Math.atan2(deltaX, -deltaY));
        return normalizeAngle(angle);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getDistanceToGoal() {
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private void updateTelemetry() {
        String locSource = USE_VISION ? ">>> VISION <<<" : ">>> ODOMETRY <<<";
        String aimStatus = autoAimEnabled ? "LOCKED" : "OFF";
        String shooterStatus = shooter.isAtTargetRPM() ? "READY" : "SPIN";

        telemetry.addData("Source", locSource + (USE_VISION && !visionValid ? " (NO DATA)" : ""));
        telemetry.addData("Auto-Aim", aimStatus);
        telemetry.addLine();

        telemetry.addLine("--- ODOMETRY ---");
        telemetry.addData("  Position", "X: %.1f  Y: %.1f", odoX, odoY);
        telemetry.addData("  Heading", "%.1f", odoHeading);
        telemetry.addLine();

        telemetry.addLine("--- VISION ---");
        if (visionValid) {
            telemetry.addData("  Position", "X: %.1f  Y: %.1f", visionX, visionY);
            telemetry.addData("  Heading", "%.1f", visionHeading);
            telemetry.addData("  Tags", vision.getTagCount());
        } else {
            telemetry.addLine("  NO DATA");
        }
        telemetry.addLine();

        telemetry.addLine("--- TARGETING ---");
        telemetry.addData("Distance", "%.1f in", getDistanceToGoal());
        telemetry.addData("Angle to Goal", "%.1f", angleToGoal);
        telemetry.addData("Turret Target", "%.1f", turret.getLastTargetFieldAngle());
        telemetry.addData("Turret Actual", "%.1f", turret.getTurretAngle());
        telemetry.addLine();

        telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), SHOOTER_RPM);
        telemetry.update();
    }
}