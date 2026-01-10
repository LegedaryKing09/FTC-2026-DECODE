package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;

/**
 * Field-Centric Turret + Shooter Test (Simplified)
 * Controls:
 *   Left Stick = Drive forward/back
 *   Right Stick = Turn
 *   A = Toggle intake
 *   B = Toggle transfer
 *   X = Toggle uptake
 *   Y = Toggle turret field tracking on/off
 *   RB = Toggle speed mode
 *   LB = Reset IMU
 *   Shooter always runs at SHOOTER_RPM (tune in Dashboard)
 */
@Config
@TeleOp(name = "Field Turret + Shooter Test", group = "Test")
public class FieldTurretShooterTest extends LinearOpMode {

    // Field target (tune in Dashboard)
    public static double TARGET_FIELD_ANGLE = 130.0;

    // Shooter RPM (tune in Dashboard)
    public static double SHOOTER_RPM = 4600.0;

    // Telemetry update interval
    public static double TELEMETRY_INTERVAL_MS = 100;

    // Controllers
    private SixWheelDriveController drive;
    private TurretController turret;
    private TurretFieldController fieldController;
    private NewShooterController shooter;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;

    // Button states for toggle detection
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();
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
            telemetry.addLine("WARNING: shooter1 not found");
        }
        try {
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addLine("(shooter2 not found)");
        }
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            telemetry.addLine("WARNING: intake not found");
        }
        intake = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            telemetry.addLine("WARNING: transfer not found");
        }
        transfer = new NewTransferController(transferMotor);

        CRServo uptakeServo1 = hardwareMap.get(CRServo.class, "servo1");
        CRServo uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        uptake = new UptakeController(uptakeServo1, uptakeServo2);

        telemetry.addLine("=== FIELD TURRET + SHOOTER ===");
        telemetry.addLine("A=Intake | B=Transfer | X=Uptake");
        telemetry.addLine("Y=Toggle Turret Tracking");
        telemetry.update();

        waitForStart();

        // Initialize at start
        drive.resetOdometry();
        turret.initialize();

        // Start shooter immediately (always spinning)
        shooter.setTargetRPM(SHOOTER_RPM);
        shooter.startShooting();

        // Start turret tracking
        fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
        fieldController.enable();

        loopTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            // Track loop time
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            avgLoopTimeMs = avgLoopTimeMs * 0.95 + loopTimeMs * 0.05;

            // === UPDATE ALL CONTROLLERS ===
            drive.updateOdometry();
            turret.update();
            shooter.update();
            intake.update();
            transfer.update();
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

            // === A = TOGGLE INTAKE ===
            if (gamepad1.a && !lastA) {
                intake.toggle();
            }
            lastA = gamepad1.a;

            // === B = TOGGLE TRANSFER ===
            if (gamepad1.b && !lastB) {
                transfer.toggle();
            }
            lastB = gamepad1.b;

            // === X = TOGGLE UPTAKE ===
            if (gamepad1.x && !lastX) {
                if (uptake != null) uptake.toggle();
            }
            lastX = gamepad1.x;

            // === Y = TOGGLE TURRET TRACKING ===
            if (gamepad1.y && !lastY) {
                if (fieldController.isEnabled()) {
                    fieldController.disable();
                    turret.stop();
                } else {
                    fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
                    fieldController.enable();
                }
            }
            lastY = gamepad1.y;

            // === LB = RESET IMU ===
            if (gamepad1.left_bumper && !lastLB) {
                drive.resetOdometry();
                fieldController.resetPID();
            }
            lastLB = gamepad1.left_bumper;

            // === RB = TOGGLE SPEED MODE ===
            if (gamepad1.right_bumper && !lastRB) {
                drive.toggleSpeedMode();
            }
            lastRB = gamepad1.right_bumper;

            // === TURRET CONTROL ===
            if (fieldController.isEnabled()) {
                fieldController.update(robotHeadingDeg);
            }

            // === TELEMETRY (reduced rate) ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();

                // Status
                String turretStatus = fieldController.isEnabled() ?
                        (fieldController.isAligned() ? "ALIGNED" : "TRACKING") : "OFF";
                String shooterStatus = shooter.isAtTargetRPM() ? "READY" : "SPIN";
            }
        }

        // Cleanup
        drive.stopDrive();
        turret.stop();
        shooter.stopShooting();
        intake.setState(false);
        intake.update();
        transfer.setState(false);
        transfer.update();
        if (uptake != null) {
            uptake.setState(false);
            uptake.update();
        }
    }
}