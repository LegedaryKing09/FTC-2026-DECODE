package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@TeleOp(name="New Champion TeleOp", group="Competition")
public class NewTeleop extends LinearOpMode {

    // Ramp angle increment (tunable via FTC Dashboard)
    public static double RAMP_INCREMENT_DEGREES = 10.0;

    // Turret angle increment for manual control
    public static double TURRET_INCREMENT_DEGREES = 5.0;

    // Turret alignment enable/disable
    public static boolean ENABLE_AUTO_ALIGNMENT = true;

    // Target AprilTag ID for turret alignment
    public static int TURRET_TARGET_TAG_ID = 20;

    // Controllers
    private TurretController turret;
    private TurretAlignmentController turretAlignment;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Drive motors
    private DcMotor lf, lb;
    private DcMotor rf, rb;

    private final ElapsedTime runtime = new ElapsedTime();

    // Button debouncing
    private boolean lastRightBumper = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftBumper = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Trigger threshold for transfer toggle
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Track shooter control mode
    private boolean shooterManualMode = false;

    // Track manual turret override
    private boolean manualTurretOverride = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        initializeHardware();
        displayInitScreen();
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize turret to 0 degrees at start
        if (turret != null) {
            turret.initialize();
            telemetry.addLine("Initializing turret to 0°...");
            telemetry.update();

            // Wait for initialization to complete (with timeout)
            ElapsedTime initTimer = new ElapsedTime();
            while (!turret.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                turret.update();
                telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngle());
                telemetry.addData("Target", "0.0°");
                telemetry.update();
                sleep(20);
            }
            telemetry.addLine("Turret initialized!");
            telemetry.update();
            sleep(500);
        }

        // Initialize ramp to 0 degrees at start
        if (ramp != null) {
            ramp.initialize();
            telemetry.addLine("Initializing ramp to 0°...");
            telemetry.update();

            // Wait for initialization to complete (with timeout)
            ElapsedTime initTimer = new ElapsedTime();
            while (!ramp.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                ramp.update();
                telemetry.addData("Ramp Angle", "%.1f°", ramp.getCurrentAngle());
                telemetry.addData("Target", "0.0°");
                telemetry.update();
                sleep(20);
            }
            telemetry.addLine("Ramp initialized!");
            telemetry.update();
            sleep(500);
        }

        // Start turret auto-alignment if enabled
        if (turretAlignment != null && ENABLE_AUTO_ALIGNMENT) {
            turretAlignment.startAlignment();
            telemetry.addLine("Turret auto-alignment ENABLED");
            telemetry.update();
            sleep(500);
        }

        while (opModeIsActive()) {
            // ========== EMERGENCY STOP ==========
            handleReverse();

            // ========== DRIVE CONTROL ==========
            handleDriveControls();

            // ========== TURRET CONTROL ==========
            handleTurretControls();

            // ========== RAMP CONTROL ==========
            handleRampControls();

            // ========== SYSTEM TOGGLES ==========
            handleSystemToggles();

            // ========== SHOOTER CONTROL ==========
            handleShooterControl();

            // ========== UPDATE ALL CONTROLLERS ==========
            updateAllSystems();

            // ========== TELEMETRY ==========
            displayTelemetry();
        }
    }

    private void initializeHardware() {
        telemetry.addLine("═══════════════════════");
        telemetry.addLine("  HARDWARE CHECK");
        telemetry.addLine("═══════════════════════");

        // Initialize drive motors
        try {
            lf = hardwareMap.get(DcMotor.class, "lf");
            lb = hardwareMap.get(DcMotor.class, "lb");
            rf = hardwareMap.get(DcMotor.class, "rf");
            rb = hardwareMap.get(DcMotor.class, "rb");

            lf.setDirection(DcMotorSimple.Direction.FORWARD);
            lb.setDirection(DcMotorSimple.Direction.FORWARD);
            rf.setDirection(DcMotorSimple.Direction.REVERSE);
            rb.setDirection(DcMotorSimple.Direction.REVERSE);

            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("✓ Drive", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Drive", "NOT FOUND");
        }

        // Initialize turret
        try {
            turret = new TurretController(this);
            telemetry.addData("✓ Turret", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Turret", "NOT FOUND: " + e.getMessage());
        }

        // Initialize turret alignment controller
        try {
            turretAlignment = new TurretAlignmentController(this, turret);
            turretAlignment.setTargetTag(TURRET_TARGET_TAG_ID);
            telemetry.addData("✓ Turret Alignment", "Initialized (Tag " + TURRET_TARGET_TAG_ID + ")");
        } catch (Exception e) {
            telemetry.addData("✗ Turret Alignment", "NOT FOUND: " + e.getMessage());
        }

        // Initialize ramp
        try {
            ramp = new NewRampController(this);
            telemetry.addData("✓ Ramp", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Ramp", "NOT FOUND: " + e.getMessage());
        }

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            telemetry.addData("✓ Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Intake", "NOT FOUND");
        }
        intake = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            telemetry.addData("✓ Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Transfer", "NOT FOUND");
        }
        transfer = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            telemetry.addData("✓ Uptake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Uptake", "NOT FOUND");
        }
        uptake = new UptakeController(uptakeServo);

        // Initialize shooter
        DcMotor shooterMotor = null;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            telemetry.addData("✓ Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter", "NOT FOUND");
        }
        shooter = new NewShooterController(shooterMotor);
    }

    private void displayInitScreen() {
        telemetry.addLine();
        telemetry.addLine("═══════════════════════");
        telemetry.addLine("    🎮 CONTROLS");
        telemetry.addLine("═══════════════════════");
        telemetry.addLine();
        telemetry.addLine("🚗 DRIVE");
        telemetry.addLine("  Left Stick Y: Forward/Back");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine();
        telemetry.addLine("🎯 TURRET");
        telemetry.addLine("  AUTO-ALIGN: Enabled (Tag " + TURRET_TARGET_TAG_ID + ")");
        telemetry.addLine("  DPAD LEFT: Rotate Left");
        telemetry.addLine("  DPAD RIGHT: Rotate Right");
        telemetry.addLine("  (Manual disables auto-align)");
        telemetry.addLine();
        telemetry.addLine("📐 RAMP (PID Control)");
        telemetry.addLine("  DPAD UP: +10° Angle");
        telemetry.addLine("  DPAD DOWN: -10° Angle");
        telemetry.addLine();
        telemetry.addLine("🎯 SHOOTER");
        telemetry.addLine("  X: Increase RPM +100");
        telemetry.addLine("  A: Decrease RPM -100");
        telemetry.addLine("  Y: SHOOT (reach target RPM)");
        telemetry.addLine("  LT: Manual power (overrides)");
        telemetry.addLine();
        telemetry.addLine("🛑 B: EMERGENCY STOP");
        telemetry.addLine();
        telemetry.addLine("📦 SYSTEMS");
        telemetry.addLine("  RB: Intake Toggle");
        telemetry.addLine("  RT: Transfer Toggle");
        telemetry.addLine("  LB: Uptake Toggle");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
    }

    /**
     * Reverse - B button
     */
    private void handleReverse() {
        boolean currentB = gamepad1.b;
        if (currentB && !lastB) {
            assert intake != null;
            if (intake.reversed) intake.toggle();
            else if (intake.isActive()) intake.reversed = true;
            else {intake.reversed = true; intake.toggle();}
            if (transfer.reversed) transfer.toggle();
            else if (transfer.isActive()) transfer.reversed = true;
            else {transfer.reversed = true; transfer.toggle();}
            if (uptake.reversed) uptake.toggle();
            else if (uptake.isActive()) uptake.reversed = true;
            else {uptake.reversed = true; uptake.toggle();}
            intake.update();
            transfer.update();
            uptake.update();
        }
        lastB = currentB;
    }

    /**
     * Handle drive controls - Left stick Y for forward/back, Right stick X for turning
     */
    private void handleDriveControls() {
        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        // Apply sensitivity curve (exponent 2.0 for smoother low-speed control)
        double drive = applySensitivityCurve(rawDrive);
        double turn = applySensitivityCurve(rawTurn);

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        if (lf != null) lf.setPower(leftPower);
        if (lb != null) lb.setPower(leftPower);
        if (rf != null) rf.setPower(rightPower);
        if (rb != null) rb.setPower(rightPower);
    }

    /**
     * Apply sensitivity curve to input
     */
    private double applySensitivityCurve(double value) {
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double curved = Math.pow(magnitude, 2.0);
        return sign * curved;
    }

    /**
     * Handle turret controls - Manual control or auto-alignment
     */
    private void handleTurretControls() {
        if (turret == null) return;

        // DPAD LEFT - Manual rotation left (increment angle)
        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !lastDpadLeft) {
            manualTurretOverride = true;
            if (turretAlignment != null) {
                turretAlignment.stopAlignment();
            }
            turret.incrementAngle(TURRET_INCREMENT_DEGREES);
        }
        lastDpadLeft = currentDpadLeft;

        // DPAD RIGHT - Manual rotation right (decrement angle)
        boolean currentDpadRight = gamepad1.dpad_right;
        if (currentDpadRight && !lastDpadRight) {
            manualTurretOverride = true;
            if (turretAlignment != null) {
                turretAlignment.stopAlignment();
            }
            turret.decrementAngle(TURRET_INCREMENT_DEGREES);
        }
        lastDpadRight = currentDpadRight;

        // If in manual mode and turret reached target, allow auto-align to resume
        if (manualTurretOverride && turret.atTarget()) {
            // Check if user wants to resume auto-align (could add a button for this)
            // For now, manual mode persists until alignment is explicitly restarted
        }

        // Run auto-alignment if enabled and not in manual override
        if (turretAlignment != null && ENABLE_AUTO_ALIGNMENT && !manualTurretOverride) {
            turretAlignment.update();
        }
    }

    /**
     * Handle ramp controls - DPAD Up/Down for precise angle adjustment with PID
     */
    private void handleRampControls() {
        if (ramp == null) return;

        // DPAD UP - Increase ramp angle by configured amount
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            ramp.incrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastDpadUp = currentDpadUp;

        // DPAD DOWN - Decrease ramp angle by configured amount
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            ramp.decrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastDpadDown = currentDpadDown;
    }

    /**
     * Handle system toggles (gamepad1)
     */
    private void handleSystemToggles() {
        // Right Bumper - Intake toggle
        boolean currentRightBumper = gamepad1.right_bumper;
        if (currentRightBumper && !lastRightBumper) {
            intake.reversed = false;
            intake.toggle();
            intake.update();
        }
        lastRightBumper = currentRightBumper;

        // Right Trigger - Transfer toggle
        boolean currentRightTrigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        if (currentRightTrigger && !lastRightTrigger) {
            transfer.reversed = false;
            transfer.toggle();
            transfer.update();
        }
        lastRightTrigger = currentRightTrigger;

        // Left Bumper - Uptake toggle
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumper) {
            uptake.reversed = false;
            uptake.toggle();
            uptake.update();
        }
        lastLeftBumper = currentLeftBumper;
    }

    /**
     * Handle shooter control - X/A for RPM, Y for shoot
     */
    private void handleShooterControl() {
        // X Button - Increase target RPM
        boolean currentX = gamepad1.x;
        if (currentX && !lastX) {
            shooter.incrementTargetRPM();
        }
        lastX = currentX;

        // A Button - Decrease target RPM
        boolean currentA = gamepad1.a;
        if (currentA && !lastA) {
            shooter.decrementTargetRPM();
        }
        lastA = currentA;

        // Y Button - Toggle shoot mode
        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            shooter.toggleShoot();
        }
        lastY = currentY;

        // Left Trigger - Manual control (overrides)
        double triggerValue = gamepad1.left_trigger;
        if (triggerValue > 0.1) {
            shooterManualMode = true;
            shooter.stopShooting();
            shooter.setPower(triggerValue);
        } else if (shooterManualMode && triggerValue <= 0.1) {
            shooterManualMode = false;
            shooter.setPower(0);
        }

        shooter.update();
    }

    private void updateAllSystems() {
        if (turret != null) turret.update();
        if (ramp != null) ramp.update();
        intake.update();
        transfer.update();
        uptake.update();
    }

    private void displayTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Battery", "%.2f V", getBatteryVoltage());

        // Drive status
        telemetry.addLine();
        telemetry.addLine("═══ DRIVE ═══");
        double leftPwr = lf != null ? lf.getPower() : 0;
        double rightPwr = rf != null ? rf.getPower() : 0;
        telemetry.addData("Left Power", "%.2f", leftPwr);
        telemetry.addData("Right Power", "%.2f", rightPwr);

        // Turret status with auto-alignment info
        if (turret != null) {
            telemetry.addLine();
            telemetry.addLine("═══ TURRET ═══");
            telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
            telemetry.addData("Error", "%.1f°", turret.getAngleError());

            String turretStatus = turret.isMoving() ? "🔄 MOVING" : (turret.atTarget() ? "✓ AT TARGET" : "IDLE");
            telemetry.addData("Status", turretStatus);

            if (turretAlignment != null) {
                String mode = manualTurretOverride ? "🟡 MANUAL" : "🟢 AUTO-ALIGN";
                telemetry.addData("Mode", mode);

                if (!manualTurretOverride && ENABLE_AUTO_ALIGNMENT) {
                    telemetry.addData("Align State", turretAlignment.getState());
                    telemetry.addData("Has Target", turretAlignment.hasTarget() ? "✓ YES" : "✗ NO");
                    if (turretAlignment.hasTarget()) {
                        telemetry.addData("TX Error", "%.2f°", turretAlignment.getTargetError());
                    }
                }
            } else {
                telemetry.addData("Mode", "🔴 MANUAL ONLY");
            }
        }

        // Ramp status (detailed PID info)
        if (ramp != null) {
            telemetry.addLine();
            telemetry.addLine("═══ RAMP (PID) ═══");
            telemetry.addData("Current Angle", "%.1f°", ramp.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", ramp.getTargetAngle());
            telemetry.addData("Error", "%.1f°", ramp.getAngleError());
            telemetry.addData("Power", "%.2f", ramp.getPower());
            String status = ramp.isMoving() ? "🔄 MOVING" : (ramp.atTarget() ? "✓ AT TARGET" : "IDLE");
            telemetry.addData("Status", status);
        }

        // Shooter status
        telemetry.addLine();
        telemetry.addLine("═══ SHOOTER ═══");
        String shootMode = shooter.isShootMode() ? "🟢 SHOOTING" : "IDLE";
        if (shooterManualMode) shootMode = "🟡 MANUAL";
        telemetry.addData("Mode", shootMode);
        telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Power", "%.2f", shooter.getCurrentPower());

        // Systems status
        telemetry.addLine();
        telemetry.addLine("═══ SYSTEMS ═══");
        telemetry.addData("Intake (RB)", "%s", intake.isActive() ? "🟢 ON" : "⚫ OFF");
        telemetry.addData("Transfer (RT)", "%s", transfer.isActive() ? "🟢 ON" : "⚫ OFF");
        telemetry.addData("Uptake (LB)", "%s", uptake.isActive() ? "🟢 ON" : "⚫ OFF");

        telemetry.addLine();
        telemetry.addLine("DPAD L/R=Turret | U/D=Ramp | X/A=RPM | Y=Shoot");

        telemetry.update();
    }

    private double getBatteryVoltage() {
        double voltage = 0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }
        return voltage;
    }
}