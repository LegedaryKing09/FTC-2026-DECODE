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

    // Controllers
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Drive motors
    private DcMotor lf, lb;
    private DcMotor rf, rb;

    private ElapsedTime runtime = new ElapsedTime();

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

    // Emergency stop flag
    private boolean emergencyStop = false;

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

        while (opModeIsActive()) {
            // ========== EMERGENCY STOP ==========
            handleEmergencyStop();

            if (!emergencyStop) {
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
            }

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
        CRServo turretServo = null;
        AnalogInput turretEncoder = null;
        try {
            turretServo = hardwareMap.get(CRServo.class, "turret");
            turretEncoder = hardwareMap.get(AnalogInput.class, "turret_analog");
            telemetry.addData("✓ Turret", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Turret", "NOT FOUND");
        }
        turret = new TurretController(turretServo, turretEncoder, runtime);

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
        telemetry.addLine("  DPAD LEFT: Turn Left");
        telemetry.addLine("  DPAD RIGHT: Turn Right");
        telemetry.addLine();
        telemetry.addLine("📐 RAMP");
        telemetry.addLine("  DPAD UP: Increase Angle");
        telemetry.addLine("  DPAD DOWN: Decrease Angle");
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
     * Handle emergency stop - B button
     */
    private void handleEmergencyStop() {
        boolean currentB = gamepad1.b;
        if (currentB && !lastB) {
            emergencyStop = !emergencyStop;
            if (emergencyStop) {
                stopAllSystems();
            }
        }
        lastB = currentB;
    }

    /**
     * Stop all motors and systems
     */
    private void stopAllSystems() {
        // Stop drive
        if (lf != null) lf.setPower(0);
        if (lb != null) lb.setPower(0);
        if (rf != null) rf.setPower(0);
        if (rb != null) rb.setPower(0);

        if (turret != null) turret.setPower(0);
        if (shooter != null) {
            shooter.stopShooting();
            shooter.setPower(0);
        }
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    /**
     * Handle drive controls - Left stick Y for forward/back, Right stick X for turning
     */
    private void handleDriveControls() {
        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        // Apply sensitivity curve (exponent 2.0 for smoother low-speed control)
        double drive = applySensitivityCurve(rawDrive, 2.0);
        double turn = applySensitivityCurve(rawTurn, 2.0);

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
    private double applySensitivityCurve(double value, double exponent) {
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double curved = Math.pow(magnitude, exponent);
        return sign * curved;
    }

    /**
     * Handle turret controls - DPAD Left/Right for manual turning
     */
    private void handleTurretControls() {
        turret.update();

        // DPAD Left - Turn turret left
        if (gamepad1.dpad_left) {
            turret.setPower(-0.5);
        }
        // DPAD Right - Turn turret right
        else if (gamepad1.dpad_right) {
            turret.setPower(0.5);
        }
        // Stop turret when neither pressed
        else {
            turret.setPower(0);
        }
    }

    /**
     * Handle ramp controls - DPAD Up/Down for angle adjustment
     */
    private void handleRampControls() {
        if (ramp == null) return;
        ramp.update();

        // DPAD UP - Increase ramp angle
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            ramp.incrementAngle(0);
        }
        lastDpadUp = currentDpadUp;

        // DPAD DOWN - Decrease ramp angle
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            ramp.decrementAngle(0);
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
            intake.toggle();
        }
        lastRightBumper = currentRightBumper;

        // Right Trigger - Transfer toggle
        boolean currentRightTrigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        if (currentRightTrigger && !lastRightTrigger) {
            transfer.toggle();
        }
        lastRightTrigger = currentRightTrigger;

        // Left Bumper - Uptake toggle
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumper) {
            uptake.toggle();
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
        intake.update();
        transfer.update();
        uptake.update();
    }

    private void displayTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Battery", "%.2f V", getBatteryVoltage());

        // Emergency stop warning
        if (emergencyStop) {
            telemetry.addLine();
            telemetry.addLine(" EMERGENCY STOP ACTIVE ");
            telemetry.addLine("Press B to resume");
            telemetry.addLine();
        }

        // Drive status
        telemetry.addLine();
        telemetry.addLine("═══ DRIVE ═══");
        double leftPwr = lf != null ? lf.getPower() : 0;
        double rightPwr = rf != null ? rf.getPower() : 0;
        telemetry.addData("Left Power", "%.2f", leftPwr);
        telemetry.addData("Right Power", "%.2f", rightPwr);

        // Turret status
        telemetry.addLine();
        telemetry.addLine("═══ TURRET ═══");
        telemetry.addData("Position", "%.2f°", turret.getCurrentPosition());

        // Ramp status
        if (ramp != null) {
            telemetry.addLine();
            telemetry.addLine("═══ RAMP ═══");
            telemetry.addData("Angle", "%.1f°", ramp.getCurrentAngle());
            telemetry.addData("Power", "%.2f", ramp.getPower());
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
        telemetry.addLine("X/A=RPM | Y=Shoot | B=E-STOP");

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