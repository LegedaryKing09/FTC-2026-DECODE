package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;

/**
 * Comprehensive Teleop - Based on December Teleop logic
 *
 * === DRIVER 1 (gamepad1) ===
 * Left Stick Y:   Drive forward/backward
 * Right Stick X:  Turn left/right
 * Right Bumper:   Toggle intake + transfer (NO uptake)
 *
 * === DRIVER 2 (gamepad2) ===
 * Right Trigger:  Hold to run uptake + disable auto-aim
 * Left Bumper:    Ramp +1 degree
 * Left Trigger:   Ramp -1 degree
 * D-Pad Right:    Turret target +1 degree
 * D-Pad Left:     Turret target -1 degree
 * Right Stick X:  Manual turret (when auto-aim off)
 *
 * Y Button:       FAR preset (4500 RPM, ramp 171°, turret 25°) + enable auto-aim
 * A Button:       CLOSE preset (3650 RPM, ramp 92°, turret 45°) + enable auto-aim
 * X Button:       Toggle auto-aim on/off
 *
 * === AUTO-AIM BEHAVIOR ===
 * - Starts OFF
 * - Turns ON when preset button pressed (Y or A)
 * - Turns OFF when RT pressed (shooting)
 * - Can be toggled manually with X
 *
 * === NOTE: UPTAKE SWITCH NOT USED ===
 * Uptake is controlled directly by driver 2 RT - no ball detection
 */
@Config
@TeleOp(name = "超级无敌高级Teleop", group = "Competition")
public class MyOnlyTeleop extends LinearOpMode {

    // === PRESETS (same as December Teleop) ===
    public static double FAR_RPM = 4500.0;
    public static double FAR_RAMP_ANGLE = 171.0;
    public static double FAR_TURRET_ANGLE = 25.0;

    public static double CLOSE_RPM = 3650.0;
    public static double CLOSE_RAMP_ANGLE = 92.0;
    public static double CLOSE_TURRET_ANGLE = 45.0;

    public static double IDLE_RPM = 2000.0;

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = 1.0;
    public static double TURRET_TARGET_INCREMENT = 1.0;
    public static double TURRET_MANUAL_SENSITIVITY = 0.3;

    // === BALL DETECTION (same as December) ===
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // === TELEMETRY ===
    public static double TELEMETRY_INTERVAL_MS = 100;

    // === CONTROLLERS ===
    private SixWheelDriveController drive;
    private TurretController turret;
    private TurretFieldController turretField;  // NEW: for auto-aim
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Ball detection switch
    private AnalogInput uptakeSwitch;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // Loop timing
    private double loopTimeMs = 0;
    private double avgLoopTimeMs = 0;

    // === GAMEPAD 1 BUTTON STATES ===
    private boolean lastRB1 = false;

    // === GAMEPAD 2 BUTTON STATES ===
    private boolean lastY2 = false;
    private boolean lastA2 = false;
    private boolean lastX2 = false;
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastLB2 = false;
    private boolean lastLT2Pressed = false;

    // Trigger threshold (same as December: 30%)
    private static final double TRIGGER_THRESHOLD = 0.3;

    // === STATE TRACKING ===
    private boolean intakeModeActive = false;
    private boolean uptakeFromTrigger = false;
    private double currentTargetRPM = 0;

    // === Auto-aim state ===
    private boolean autoAimEnabled = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeHardware();

        telemetry.addLine("=== COMPREHENSIVE TELEOP ===");
        telemetry.addLine("Based on December Teleop logic");
        telemetry.addLine("Hardware initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize turret angle tracking AFTER start (same as December)
        if (turret != null) {
            turret.initialize();
        }

        // Initialize ramp to 0 degrees at start (same as December)
        if (ramp != null) {
            ramp.initialize();
            ElapsedTime initTimer = new ElapsedTime();
            while (!ramp.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                ramp.update();
                sleep(20);
            }
        }

        // Auto-aim starts DISABLED
        if (turretField != null) {
            turretField.disable();
        }

        // Start shooter at idle RPM (2000)
        if (shooter != null) {
            shooter.setTargetRPM(IDLE_RPM);
            currentTargetRPM = IDLE_RPM;
            shooter.startShooting();
        }

        loopTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            // Track loop time (same as December)
            loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            avgLoopTimeMs = avgLoopTimeMs * 0.95 + loopTimeMs * 0.05;

            // === FAST UPDATES (every loop) ===
            handleDriveControls();
            handleDriver1Controls();
            handleDriver2Controls();

            // NOTE: Uptake switch check removed - uptake controlled directly by driver 2 RT

            // Update all controllers
            updateAllSystems();

            // === SLOW UPDATES (telemetry) ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                updateTelemetry();
            }
        }

        // Cleanup
        if (shooter != null) shooter.stopShooting();
        if (turretField != null) turretField.disable();
    }

    /**
     * Initialize hardware (same as December Teleop)
     */
    private void initializeHardware() {
        // Drive controller
        try {
            drive = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addLine("WARNING: Drive controller failed to initialize");
        }

        // Turret
        try {
            turret = new TurretController(this);
            turretField = new TurretFieldController(turret);  // NEW
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Turret: " + e.getMessage());
        }

        // Ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Ramp: " + e.getMessage());
        }

        // Intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Intake: " + e.getMessage());
        }
        intake = new NewIntakeController(intakeMotor);

        // Transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Transfer: " + e.getMessage());
        }
        transfer = new NewTransferController(transferMotor);

        // Uptake
        CRServo uptakeServo = null;
        CRServo uptakeServo2 = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            uptakeServo2 = hardwareMap.get(CRServo.class, "uptake2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake: " + e.getMessage());
        }
        uptake = new UptakeController(uptakeServo, uptakeServo2);

        // Uptake ball detection switch (same name as December: "uptakeSwitch")
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake Switch: " + e.getMessage());
        }

        // Shooter
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Shooter: " + e.getMessage());
        }
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);
    }

    /**
     * Sensitivity curve - CUBIC (same as December Teleop)
     */
    private double applySensitivityCurve(double input) {
        return input * input * input;
    }

    /**
     * Drive controls - ALWAYS FAST MODE
     */
    private void handleDriveControls() {
        if (drive == null) return;

        // Update odometry
        drive.updateOdometry();

        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        // Always use FAST speed multipliers
        double speedMult = SixWheelDriveController.FAST_SPEED_MULTIPLIER;
        double turnMult = SixWheelDriveController.FAST_TURN_MULTIPLIER;

        // Apply sensitivity curve and multipliers
        double drivePower = applySensitivityCurve(rawDrive) * speedMult;
        double turnPower = applySensitivityCurve(rawTurn) * turnMult;

        // Use arcade drive
        drive.arcadeDrive(drivePower, turnPower);
    }

    /**
     * Driver 1 controls
     */
    private void handleDriver1Controls() {
        // Right bumper - toggle intake mode (same logic as December)
        boolean currentRB1 = gamepad1.right_bumper;
        if (currentRB1 && !lastRB1) {
            if (!intakeModeActive) {
                startIntakeMode();
            } else {
                stopIntakeMode();
            }
        }
        lastRB1 = currentRB1;
    }

    /**
     * Start intake mode - ONLY intake + transfer (uptake controlled separately by driver 2)
     */
    private void startIntakeMode() {
        intakeModeActive = true;

        if (intake != null && !intake.isActive()) {
            intake.reversed = false;
            intake.toggle();
        }
        if (transfer != null && !transfer.isActive()) {
            transfer.reversed = false;
            transfer.toggle();
        }
        // NOTE: Uptake is NOT started here - controlled by driver 2 RT
    }

    /**
     * Stop intake mode - ONLY intake + transfer
     */
    private void stopIntakeMode() {
        intakeModeActive = false;
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        // NOTE: Uptake is NOT stopped here - controlled by driver 2 RT
    }

    /**
     * Driver 2 controls
     */
    private void handleDriver2Controls() {

        // === RT: HOLD TO RUN UPTAKE + DISABLE AUTO-AIM ===
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // Run uptake while held
            if (uptake != null && !uptake.isActive()) {
                uptake.reversed = false;
                uptake.toggle();
                uptakeFromTrigger = true;
            }

            // Disable auto-aim when shooting
            if (autoAimEnabled) {
                autoAimEnabled = false;
                if (turretField != null) turretField.disable();
            }
        } else {
            // Stop uptake when released
            if (uptakeFromTrigger && uptake != null && uptake.isActive()) {
                uptake.toggle();
                uptakeFromTrigger = false;
            }
        }

        // === LB: RAMP +1° ===
        boolean currentLB2 = gamepad2.left_bumper;
        if (currentLB2 && !lastLB2) {
            if (ramp != null) ramp.incrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastLB2 = currentLB2;

        // === LT: RAMP -1° ===
        boolean ltPressed = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (ltPressed && !lastLT2Pressed) {
            if (ramp != null) ramp.decrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastLT2Pressed = ltPressed;

        // === DPAD: TURRET TARGET ANGLE ===
        boolean currentDpadRight2 = gamepad2.dpad_right;
        if (currentDpadRight2 && !lastDpadRight2) {
            if (turretField != null) {
                turretField.setTargetFieldAngle(turretField.getTargetFieldAngle() + TURRET_TARGET_INCREMENT);
            }
        }
        lastDpadRight2 = currentDpadRight2;

        boolean currentDpadLeft2 = gamepad2.dpad_left;
        if (currentDpadLeft2 && !lastDpadLeft2) {
            if (turretField != null) {
                turretField.setTargetFieldAngle(turretField.getTargetFieldAngle() - TURRET_TARGET_INCREMENT);
            }
        }
        lastDpadLeft2 = currentDpadLeft2;

        // === RIGHT STICK X: MANUAL TURRET (when auto-aim off) ===
        if (!autoAimEnabled && turret != null) {
            double turretInput = gamepad2.right_stick_x;
            if (Math.abs(turretInput) > 0.1) {
                turret.setPower(turretInput * TURRET_MANUAL_SENSITIVITY);
            } else {
                turret.stop();
            }
        }

        // === Y: FAR PRESET + ENABLE AUTO-AIM ===
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            if (shooter != null) {
                shooter.setTargetRPM(FAR_RPM);
                currentTargetRPM = FAR_RPM;
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);
            if (turretField != null) {
                turretField.setTargetFieldAngle(FAR_TURRET_ANGLE);
                turretField.enable();
                autoAimEnabled = true;
            }
        }
        lastY2 = currentY2;

        // === A: CLOSE PRESET + ENABLE AUTO-AIM ===
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            if (shooter != null) {
                shooter.setTargetRPM(CLOSE_RPM);
                currentTargetRPM = CLOSE_RPM;
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(CLOSE_RAMP_ANGLE);
            if (turretField != null) {
                turretField.setTargetFieldAngle(CLOSE_TURRET_ANGLE);
                turretField.enable();
                autoAimEnabled = true;
            }
        }
        lastA2 = currentA2;

        // === X: TOGGLE AUTO-AIM ===
        boolean currentX2 = gamepad2.x;
        if (currentX2 && !lastX2) {
            autoAimEnabled = !autoAimEnabled;
            if (turretField != null) {
                if (autoAimEnabled) {
                    turretField.enable();
                } else {
                    turretField.disable();
                }
            }
            if (!autoAimEnabled && turret != null) {
                turret.stop();
            }
        }
        lastX2 = currentX2;
    }

    /**
     * Update all subsystem controllers
     */
    private void updateAllSystems() {
        if (turret != null) turret.update();

        // NEW: Update turret field controller for auto-aim
        if (turretField != null && autoAimEnabled && drive != null) {
            turretField.update(drive.getHeadingDegrees());
        }

        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }

    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        // Status line
        String intakeStatus = intakeModeActive ? "INTAKE" : "OFF";
        String uptakeStatus = (uptake != null && uptake.isActive()) ? "UPTAKE" : "";
        String shooterStatus = shooter != null && shooter.isShootMode() ?
                (shooter.isAtTargetRPM() ? "READY" : "SPIN") : "OFF";
        String aimStatus = autoAimEnabled ?
                (turretField != null && turretField.isLocked() ? "LOCKED" : "AIM") : "MANUAL";

        telemetry.addData("Status", "%s %s | Shooter: %s | Turret: %s", intakeStatus, uptakeStatus, shooterStatus, aimStatus);
        telemetry.addData("Loop", "%.1fms (%.0fHz)", avgLoopTimeMs, 1000.0 / avgLoopTimeMs);
        telemetry.addLine();

        // Drive info
        if (drive != null) {
            telemetry.addData("Position", "X:%.1f Y:%.1f", drive.getX(), drive.getY());
            telemetry.addData("Heading", "%.1f°", drive.getHeadingDegrees());
        }

        // Shooter info
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
        }

        // Ramp info
        if (ramp != null) {
            telemetry.addData("Ramp", "%.1f° → %.1f°", ramp.getCurrentAngle(), ramp.getTargetAngle());
        }

        // Turret info
        if (turret != null) {
            telemetry.addData("Turret", "%.1f°", turret.getTurretAngle());
        }
        if (turretField != null && autoAimEnabled) {
            telemetry.addData("Turret Target", "%.1f° (err: %.1f°)",
                    turretField.getTargetFieldAngle(), turretField.getFieldError());
        }

        // Uptake status
        telemetry.addData("Uptake", uptake != null && uptake.isActive() ? "RUNNING" : "off");

        telemetry.update();
    }
}