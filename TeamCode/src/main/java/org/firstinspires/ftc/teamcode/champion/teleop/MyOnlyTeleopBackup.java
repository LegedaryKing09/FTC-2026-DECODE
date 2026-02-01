package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;


/**
 * BACKUP Teleop - Manual Turret Only (No Auto-Aim)
 *
 * Use this if auton data transfer failed or auto-aim isn't working.
 * All turret control is manual via right stick.
 *
 * === DRIVER 1 (gamepad1) - ARCADE DRIVE ===
 * Left Stick Y:   Forward/Backward
 * Right Stick X:  Turn Left/Right
 * Right Bumper:   Toggle intake + transfer + uptake
 * Left Bumper:    HOLD to VOMIT (reverse all)
 *
 * === DRIVER 2 (gamepad2) ===
 * Right Trigger:  HOLD to SHOOT (bypass ball detection)
 * Right Stick X:  Manual turret control
 *
 * Left Bumper:    RETRACT ramp
 * Left Trigger:   EXTEND ramp
 *
 * D-Pad Up:       RPM +100
 * D-Pad Down:     RPM -100
 *
 * Y Button:       FAR preset (RPM + ramp only, NO auto-aim)
 * A Button:       CLOSE preset (RPM + ramp only, NO auto-aim)
 * X Button:       Set RPM to IDLE
 */
@Config
@TeleOp(name = "Undefeated Backup Teleop", group = "Competition")
public class MyOnlyTeleopBackup extends LinearOpMode {

    // === PRESETS ===
    public static double FAR_RPM = 4300.0;
    public static double FAR_RAMP_ANGLE = 0.7;

    public static double CLOSE_RPM = 3600.0;
    public static double CLOSE_RAMP_ANGLE = 0.51;

    public static double IDLE_RPM = 2000.0;

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT = 0.05;
    public static double RPM_INCREMENT = 100.0;
    public static double TURRET_MANUAL_SENSITIVITY = 0.5;

    // === BALL DETECTION ===
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // === TELEMETRY ===
    public static double TELEMETRY_INTERVAL_MS = 100;

    // === CONTROLLERS ===
    private SixWheelDriveController drive;
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Ball detection switch
    private AnalogInput uptakeSwitch;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // === GAMEPAD 1 BUTTON STATES ===
    private boolean lastRB1 = false;

    // === GAMEPAD 2 BUTTON STATES ===
    private boolean lastY2 = false;
    private boolean lastA2 = false;
    private boolean lastX2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastLB2 = false;
    private boolean lastLT2Pressed = false;

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.2;

    // === STATE TRACKING ===
    private boolean intakeModeActive = false;
    private boolean allSystemsFromTrigger = false;
    private double currentTargetRPM = 0;

    // === Preset mode tracking ===
    private enum PresetMode { IDLE, FAR, CLOSE }
    private PresetMode currentPresetMode = PresetMode.IDLE;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== BACKUP TELEOP ===");
        telemetry.addLine("Manual turret control only");
        telemetry.addLine("No auto-aim, no position tracking");
        telemetry.update();

        initializeHardware();

        sleep(300);

        waitForStart();
        runtime.reset();

        // Initialize turret at current position as 0°
        if (turret != null) {
            turret.initialize();
        }

        // Initialize ramp
        if (ramp != null) {
            ramp.initialize();
        }

        // Start shooter at idle RPM
        if (shooter != null) {
            shooter.setTargetRPM(IDLE_RPM);
            currentTargetRPM = IDLE_RPM;
            shooter.startShooting();
        }

        telemetryTimer.reset();

        while (opModeIsActive()) {
            // Handle controls
            handleDriveControls();
            handleDriver1Controls();
            handleDriver2Controls();

            // Ball detection (when intake mode active and RT not held)
            boolean bypassBallDetection = gamepad2.right_trigger > TRIGGER_THRESHOLD;
            if (intakeModeActive && uptakeSwitch != null && !bypassBallDetection) {
                double switchVoltage = uptakeSwitch.getVoltage();
                if (switchVoltage < UPTAKE_SWITCH_THRESHOLD) {
                    if (uptake != null && uptake.isActive()) {
                        uptake.toggle();
                    }
                } else {
                    if (uptake != null && !uptake.isActive()) {
                        uptake.reversed = false;
                        uptake.toggle();
                    }
                }
            }

            // Update all systems
            updateAllSystems();

            // Update telemetry periodically
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                updateTelemetry();
            }
        }

        // Cleanup
        if (shooter != null) shooter.stopShooting();
    }

    private void initializeHardware() {
        // Drive controller
        try {
            drive = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addData("Drive", "FAILED: " + e.getMessage());
        }

        // Turret
        try {
            turret = new TurretController(this);
        } catch (Exception e) {
            telemetry.addData("Turret", "FAILED: " + e.getMessage());
        }

        // Intake
        try {
            DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intake = new NewIntakeController(intakeMotor);
        } catch (Exception e) {
            telemetry.addData("Intake", "FAILED");
        }

        // Transfer
        try {
            DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            transfer = new NewTransferController(transferMotor);
        } catch (Exception e) {
            telemetry.addData("Transfer", "FAILED");
        }

        // Uptake
        try {
            CRServo servo1 = hardwareMap.get(CRServo.class, "servo1");
            CRServo servo2 = hardwareMap.get(CRServo.class, "servo2");
            uptake = new UptakeController(servo1, servo2);
        } catch (Exception e) {
            telemetry.addData("Uptake", "FAILED");
        }

        // Uptake switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            // Optional
        }

        // Shooter
        try {
            DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooter = new NewShooterController(shooterMotor);
        } catch (Exception e) {
            telemetry.addData("Shooter", "FAILED");
        }

        // Ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception e) {
            telemetry.addData("Ramp", "FAILED");
        }

        telemetry.update();
    }

    private void handleDriveControls() {
        if (drive == null) return;

        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        drive.arcadeDrive(forward, turn);
    }

    private void handleDriver1Controls() {
        // === RB: TOGGLE INTAKE MODE ===
        boolean currentRB1 = gamepad1.right_bumper;
        if (currentRB1 && !lastRB1) {
            intakeModeActive = !intakeModeActive;
            if (intakeModeActive) {
                if (intake != null) { intake.reversed = false; intake.toggle(); }
                if (transfer != null) { transfer.reversed = false; transfer.toggle(); }
                if (uptake != null) { uptake.reversed = false; uptake.toggle(); }
            } else {
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
            }
        }
        lastRB1 = currentRB1;

        // === LB: HOLD TO VOMIT ===
        if (gamepad1.left_bumper) {
            if (intake != null) { intake.reversed = true; if (!intake.isActive()) intake.toggle(); }
            if (transfer != null) { transfer.reversed = true; if (!transfer.isActive()) transfer.toggle(); }
            if (uptake != null) { uptake.reversed = true; if (!uptake.isActive()) uptake.toggle(); }
        } else {
            if (intake != null && intake.reversed) { intake.reversed = false; if (intake.isActive()) intake.toggle(); }
            if (transfer != null && transfer.reversed) { transfer.reversed = false; if (transfer.isActive()) transfer.toggle(); }
            if (uptake != null && uptake.reversed) { uptake.reversed = false; if (uptake.isActive()) uptake.toggle(); }
        }
    }

    private void handleDriver2Controls() {
        // === RT: HOLD TO SHOOT ===
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            if (!allSystemsFromTrigger) {
                if (intake != null && !intake.isActive()) { intake.reversed = false; intake.toggle(); }
                if (transfer != null && !transfer.isActive()) { transfer.reversed = false; transfer.toggle(); }
                if (uptake != null && !uptake.isActive()) { uptake.reversed = false; uptake.toggle(); }
                allSystemsFromTrigger = true;
            }
        } else {
            if (allSystemsFromTrigger) {
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
                allSystemsFromTrigger = false;
            }
        }

        // === RIGHT STICK X: MANUAL TURRET ===
        double turretInput = gamepad2.right_stick_x;
        if (turret != null) {
            if (Math.abs(turretInput) > 0.1) {
                turret.setPower(turretInput * TURRET_MANUAL_SENSITIVITY);
            } else {
                turret.stop();
            }
        }

        // === LB: RETRACT RAMP ===
        boolean currentLB2 = gamepad2.left_bumper;
        if (currentLB2 && !lastLB2) {
            if (ramp != null) {
                ramp.setTargetAngle(ramp.getTargetAngle() - RAMP_INCREMENT);
            }
        }
        lastLB2 = currentLB2;

        // === LT: EXTEND RAMP ===
        boolean ltPressed = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (ltPressed && !lastLT2Pressed) {
            if (ramp != null) {
                ramp.setTargetAngle(ramp.getTargetAngle() + RAMP_INCREMENT);
            }
        }
        lastLT2Pressed = ltPressed;

        // === DPAD UP: RPM +100 ===
        boolean currentDpadUp2 = gamepad2.dpad_up;
        if (currentDpadUp2 && !lastDpadUp2) {
            currentTargetRPM += RPM_INCREMENT;
            if (shooter != null) shooter.setTargetRPM(currentTargetRPM);
        }
        lastDpadUp2 = currentDpadUp2;

        // === DPAD DOWN: RPM -100 ===
        boolean currentDpadDown2 = gamepad2.dpad_down;
        if (currentDpadDown2 && !lastDpadDown2) {
            currentTargetRPM -= RPM_INCREMENT;
            if (currentTargetRPM < IDLE_RPM) currentTargetRPM = IDLE_RPM;
            if (shooter != null) shooter.setTargetRPM(currentTargetRPM);
        }
        lastDpadDown2 = currentDpadDown2;

        // === Y: FAR PRESET (RPM + ramp only, NO auto-aim) ===
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            currentPresetMode = PresetMode.FAR;
            currentTargetRPM = FAR_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(FAR_RPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);
            // NO turret auto-aim - driver controls manually
        }
        lastY2 = currentY2;

        // === A: CLOSE PRESET (RPM + ramp only, NO auto-aim) ===
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            currentPresetMode = PresetMode.CLOSE;
            currentTargetRPM = CLOSE_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(CLOSE_RPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(CLOSE_RAMP_ANGLE);
            // NO turret auto-aim - driver controls manually
        }
        lastA2 = currentA2;

        // === X: SET RPM TO IDLE ===
        boolean currentX2 = gamepad2.x;
        if (currentX2 && !lastX2) {
            currentPresetMode = PresetMode.IDLE;
            currentTargetRPM = IDLE_RPM;
            if (shooter != null) shooter.setTargetRPM(IDLE_RPM);
        }
        lastX2 = currentX2;
    }

    private void updateAllSystems() {
        if (drive != null) drive.updateOdometry();
        if (turret != null) turret.update();
        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== BACKUP TELEOP ===");
        telemetry.addData("Mode", currentPresetMode);

        // Shooter
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
        }

        // Ramp
        if (ramp != null) {
            telemetry.addData("Ramp", "%.3f", ramp.getTargetAngle());
        }

        // Turret
        if (turret != null) {
            telemetry.addData("Turret", "%.1f° (MANUAL)", turret.getTurretAngle());
        }

        // Intake status
        telemetry.addData("Intake Mode", intakeModeActive ? "ON" : "OFF");

        telemetry.update();
    }
}