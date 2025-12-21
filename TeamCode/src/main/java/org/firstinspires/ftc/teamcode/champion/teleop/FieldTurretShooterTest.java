package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;

/**
 * Field-Centric Turret + Shooter Test (Optimized)
 */
@Config
@TeleOp(name = "Field Turret + Shooter Test", group = "Test")
public class FieldTurretShooterTest extends LinearOpMode {

    // Field target
    public static double TARGET_FIELD_ANGLE = 45.0; // needs tuning with limelight
    public static double MANUAL_TURRET_POWER = 0.4;

    // Shooter presets
    public static double SHOOTER_RPM = 4600.0;
    public static double RPM_STEP = 100.0;
    public static boolean HOLD_TO_SHOOT = false;

    // Telemetry update interval (ms)
    public static double TELEMETRY_INTERVAL_MS = 100; // For PID update more frequent

    // Controllers
    private SixWheelDriveController drive;
    private TurretController turret;
    private TurretFieldController fieldController;
    private NewShooterController shooter;
    private UptakeController uptake;

    // Button states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLB = false;
    private boolean lastRB = false;
    private boolean lastBack = false;
    private boolean lastStart = false;
    private boolean lastRTrigger = false;

    private static final double TRIGGER_THRESHOLD = 0.3;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private double loopTimeMs = 0;
    private double avgLoopTimeMs = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive and turret
        drive = new SixWheelDriveController(this);
        turret = new TurretController(this);
        fieldController = new TurretFieldController(turret);

        // Initialize shooter
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
        } catch (Exception e) {
            telemetry.addLine("WARNING: shooter not found");
        }
        try {
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addLine("(shooter2 not found - single motor mode)");
        }
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);
        shooter.setTargetRPM(SHOOTER_RPM);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
        } catch (Exception e) {
            telemetry.addLine("WARNING: uptake not found");
        }
        uptake = new UptakeController(uptakeServo);
        telemetry.update();

        waitForStart();

        // Initialize once at start
        drive.resetOdometry();
        turret.initialize();

        fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
        fieldController.enable();

        loopTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            // Track loop time
            loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            avgLoopTimeMs = avgLoopTimeMs * 0.95 + loopTimeMs * 0.05;  // Smoothed average

            // === UPDATE SENSORS (every loop - fast!) ===
            drive.updateOdometry();
            turret.update();
            shooter.update();
            if (uptake != null) uptake.update();

            double robotHeadingDeg = drive.getHeadingDegrees();

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

            // === TURRET MODE CONTROL ===
            if (gamepad1.a) {
                fieldController.enable();
            }
            if (gamepad1.b) {
                fieldController.disable();
            }

            // === ADJUST FIELD TARGET ===
            if (gamepad1.dpad_up && !lastDpadUp) {
                TARGET_FIELD_ANGLE += 10.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                TARGET_FIELD_ANGLE -= 10.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            lastDpadDown = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !lastDpadRight) {
                TARGET_FIELD_ANGLE += 1.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            lastDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastDpadLeft) {
                TARGET_FIELD_ANGLE -= 1.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            lastDpadLeft = gamepad1.dpad_left;

            // === RESET IMU ONLY ===
            if (gamepad1.left_bumper && !lastLB) {
                drive.resetOdometry();
                fieldController.resetPID();
            }
            lastLB = gamepad1.left_bumper;

            // === SPEED MODE ===
            if (gamepad1.right_bumper && !lastRB) {
                drive.toggleSpeedMode();
            }
            lastRB = gamepad1.right_bumper;

            // === TURRET CONTROL ===
            double turretPower = 0;
            if (fieldController.isEnabled()) {
                turretPower = fieldController.update(robotHeadingDeg);
            } else {
                if (gamepad1.y) {
                    turretPower = MANUAL_TURRET_POWER;
                    turret.setPower(turretPower);
                } else if (gamepad1.x) {
                    turretPower = -MANUAL_TURRET_POWER;
                    turret.setPower(turretPower);
                } else {
                    turret.stop();
                }
            }

            // === SHOOTER CONTROL (always running) ===
            if (!shooter.isShootMode()) {
                shooter.startShooting();
            }

            // === UPTAKE CONTROL (RT = feed ball to shoot) ===
            boolean rtPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean ltPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;

            if (HOLD_TO_SHOOT) {
                if (rtPressed) {
                    if (uptake != null && !uptake.isActive()) {
                        uptake.reversed = false;
                        uptake.toggle();
                    }
                } else {
                    if (uptake != null && uptake.isActive()) {
                        uptake.toggle();
                    }
                }
            } else {
                if (rtPressed && !lastRTrigger) {
                    if (uptake != null) {
                        uptake.reversed = false;
                        if (!uptake.isActive()) uptake.toggle();
                    }
                }
                if (ltPressed) {
                    if (uptake != null && uptake.isActive()) {
                        uptake.toggle();
                    }
                }
            }
            lastRTrigger = rtPressed;

            // RPM adjustment
            if (gamepad1.back && !lastBack) {
                SHOOTER_RPM = Math.max(0, SHOOTER_RPM - RPM_STEP);
                shooter.setTargetRPM(SHOOTER_RPM);
            }
            lastBack = gamepad1.back;

            if (gamepad1.start && !lastStart) {
                SHOOTER_RPM = Math.min(NewShooterController.MAX_RPM, SHOOTER_RPM + RPM_STEP);
                shooter.setTargetRPM(SHOOTER_RPM);
            }
            lastStart = gamepad1.start;

            // === TELEMETRY (only every TELEMETRY_INTERVAL_MS) ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();

                // Status summary
                String turretStatus = fieldController.isEnabled() ?
                        (fieldController.isAligned() ? "ALIGNED" : "TRACKING") : "MANUAL";
                String shooterStatus = shooter.isShootMode() ?
                        (shooter.isAtTargetRPM() ? "READY" : "SPIN UP") : "OFF";
                String uptakeStatus = (uptake != null && uptake.isActive()) ? "FEED" : "WAIT";

                telemetry.addData("Status", "%s | %s | %s", turretStatus, shooterStatus, uptakeStatus);
                telemetry.addData("Loop", "%.1fms (%.0fHz)", avgLoopTimeMs, 1000.0 / avgLoopTimeMs);
                telemetry.addLine();

                telemetry.addData("Field Target", "%.1f", TARGET_FIELD_ANGLE);
                telemetry.addData("Field Error", "%.1f", fieldController.getFieldError());
                telemetry.addLine();

                telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), SHOOTER_RPM);
                telemetry.addData("RPM Error", "%.0f", shooter.getRPMError());
                telemetry.addLine();

                telemetry.addData("Robot", "%.1f", robotHeadingDeg);
                telemetry.addData("Turret", "%.1f", turret.getTurretAngle());
                telemetry.addLine();

                telemetry.addLine("RT=Shoot | LT=Stop | DPad=Aim");
                telemetry.update();
            }
        }

        // Cleanup
        drive.stopDrive();
        turret.stop();
        shooter.stopShooting();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }
}