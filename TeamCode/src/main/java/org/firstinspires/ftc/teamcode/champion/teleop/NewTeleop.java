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

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@TeleOp(name="Champion TeleOp", group="Competition")
public class NewTeleop extends LinearOpMode {

    // Controllers
    private EightWheelDrive drive;
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;

    private ElapsedTime runtime = new ElapsedTime();

    // Button debouncing
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastY = false;
    private boolean lastBack = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        // Initialize hardware and controllers
        initializeHardware();

        displayInitScreen();
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // ========== DRIVE CONTROL ==========
            handleDriveControls();

            // ========== TURRET CONTROL ==========
            handleTurretControls();

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

    /**
     * Initialize all hardware and create controllers
     */
    private void initializeHardware() {
        telemetry.addLine("═══════════════════════");
        telemetry.addLine("  HARDWARE CHECK");
        telemetry.addLine("═══════════════════════");

        // Initialize drive motors
        DcMotor lf = null, lb = null, rf = null, rb = null;
        try {
            lf = hardwareMap.get(DcMotor.class, "lf");
            telemetry.addData("✓ Left Front", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Left Front", "NOT FOUND");
        }

        try {
            lb = hardwareMap.get(DcMotor.class, "lb");
            telemetry.addData("✓ Left Back", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Left Back", "NOT FOUND");
        }

        try {
            rf = hardwareMap.get(DcMotor.class, "rf");
            telemetry.addData("✓ Right Front", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Right Front", "NOT FOUND");
        }

        try {
            rb = hardwareMap.get(DcMotor.class, "rb");
            telemetry.addData("✓ Right Back", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Right Back", "NOT FOUND");
        }

        drive = new EightWheelDrive(lf, lb, rf, rb);

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

    /**
     * Display initialization screen
     */
    private void displayInitScreen() {
        telemetry.addLine();
        telemetry.addLine("═══════════════════════");
        telemetry.addLine("    🎮 CONTROLS");
        telemetry.addLine("═══════════════════════");
        telemetry.addLine();
        telemetry.addLine("🚗 DRIVE (Left Stick)");
        telemetry.addLine("  Y-Axis: Forward/Back");
        telemetry.addLine("  (Sensitivity curves enabled)");
        telemetry.addLine();
        telemetry.addLine("🎯 TURRET (Right Stick)");
        telemetry.addLine("  X-Axis: Manual/Turn");
        telemetry.addLine("  A: Rotate Clockwise");
        telemetry.addLine("  B: Rotate CounterClockwise");
        telemetry.addLine("  D-Pad: Presets");
        telemetry.addLine("  X: Stop/Cancel");
        telemetry.addLine();
        telemetry.addLine("📦 SYSTEMS");
        telemetry.addLine("  RB: Intake Toggle");
        telemetry.addLine("  LB: Uptake Toggle");
        telemetry.addLine("  Y: Transfer Toggle");
        telemetry.addLine("  LT: Shooter (Variable)");
        telemetry.addLine();
        telemetry.addLine("⚙️ OTHER");
        telemetry.addLine("  Back: Speed Mode Toggle");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
    }

    /**
     * Handle drive controls (gamepad1)
     */
    private void handleDriveControls() {
        // Get joystick inputs
        double rawDrive = -gamepad1.left_stick_y;  // Inverted
        double rawTurn = gamepad1.right_stick_x;

        // Update drive
        drive.drive(rawDrive, rawTurn);

        // Toggle speed mode with back button
        boolean currentBack = gamepad1.back;
        if (currentBack && !lastBack) {
            drive.toggleSpeedMode();
        }
        lastBack = currentBack;
    }

    /**
     * Handle turret controls (gamepad1)
     * Note: Right stick X is shared between turning and turret
     * When turning is needed, it takes priority. When centered, turret can use it.
     */
    private void handleTurretControls() {
        // Update turret position
        turret.update();

        // A Button - Continuous clockwise rotation (toggle)
        boolean currentA = gamepad1.a;
        if (currentA && !lastA) {
            if (turret.isRotating() && turret.getRotationMode().equals("CLOCKWISE")) {
                turret.stopRotation();
            } else {
                turret.setClockwiseRotation();
            }
        }
        lastA = currentA;

        // B Button - Continuous counterclockwise rotation (toggle)
        boolean currentB = gamepad1.b;
        if (currentB && !lastB) {
            if (turret.isRotating() && turret.getRotationMode().equals("COUNTERCLOCKWISE")) {
                turret.stopRotation();
            } else {
                turret.setCounterclockwiseRotation();
            }
        }
        lastB = currentB;

        // D-Pad preset positions
        if (gamepad1.dpad_up) {
            turret.setTargetPosition(turret.frontPosition);
        } else if (gamepad1.dpad_left) {
            turret.setTargetPosition(turret.leftPosition);
        } else if (gamepad1.dpad_down) {
            turret.setTargetPosition(turret.backPosition);
        } else if (gamepad1.dpad_right) {
            turret.setTargetPosition(turret.rightPosition);
        }

        // X button - Stop/Cancel all turret movement
        boolean currentX = gamepad1.x;
        if (currentX && !lastX) {
            turret.cancelPositionMode();
        }
        lastX = currentX;

        // Manual control with gamepad1 right stick X
        // Note: This is shared with turning, so turret manual control
        // will also turn the robot. Use A/B buttons or D-pad for independent turret control.
        double turretManual = gamepad1.right_stick_x;
        double turretPower = turret.calculatePower(turretManual);
        turret.setPower(turretPower);
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

        // Y Button - Transfer toggle
        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            transfer.toggle();
        }
        lastY = currentY;

        // Left Bumper - Uptake toggle
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumper) {
            uptake.toggle();
        }
        lastLeftBumper = currentLeftBumper;
    }

    /**
     * Handle shooter control (gamepad1)
     */
    private void handleShooterControl() {
        double triggerValue = gamepad1.left_trigger;
        shooter.setPower(triggerValue);
    }

    /**
     * Update all systems
     */
    private void updateAllSystems() {
        intake.update();
        transfer.update();
        uptake.update();
        // Drive and turret are already updated in their respective methods
        // Shooter is directly controlled
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        telemetry.addData("⏱ Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("🔋 Battery", "%.2f V", getBatteryVoltage());
        telemetry.addLine();

        // Drive status
        telemetry.addLine("═══ 🚗 DRIVE ═══");
        telemetry.addData("Mode", drive.isFastMode() ? "⚡ FAST" : "🐢 SLOW");
        telemetry.addData("Left Power", "%.2f", drive.getLeftPower());
        telemetry.addData("Right Power", "%.2f", drive.getRightPower());
        telemetry.addLine();

        // Turret status
        telemetry.addLine("═══ 🎯 TURRET ═══");
        String turretMode = "MANUAL";
        if (turret.isPositionMode()) {
            turretMode = "AUTO POSITION";
        } else if (turret.isRotating()) {
            turretMode = "ROTATING " + turret.getRotationMode();
        }
        telemetry.addData("Mode", turretMode);
        telemetry.addData("Position", "%.2f°", turret.getCurrentPosition());
        telemetry.addData("Voltage", "%.3f V", turret.getRawVoltage());

        if (turret.isPositionMode()) {
            telemetry.addData("Target", "%.2f°", turret.getTargetPosition());
        }

        // Show which preset we're at
        String preset = "";
        if (turret.isNear(turret.frontPosition)) preset = "FRONT ✓";
        else if (turret.isNear(turret.leftPosition)) preset = "LEFT ✓";
        else if (turret.isNear(turret.backPosition)) preset = "BACK ✓";
        else if (turret.isNear(turret.rightPosition)) preset = "RIGHT ✓";
        if (!preset.isEmpty()) {
            telemetry.addData("At", preset);
        }
        telemetry.addLine();

        // Systems status
        telemetry.addLine("═══ 📦 SYSTEMS ═══");
        telemetry.addData("Intake", "%s %.2f",
                intake.isActive() ? "🟢" : "⚫", intake.getCurrentPower());
        telemetry.addData("Transfer", "%s %.2f",
                transfer.isActive() ? "🟢" : "⚫", transfer.getCurrentPower());
        telemetry.addData("Uptake", "%s %.2f",
                uptake.isActive() ? "🟢" : "⚫", uptake.getCurrentPower());
        telemetry.addData("Shooter", "%.2f (LT: %.2f)",
                shooter.getCurrentPower(), gamepad1.left_trigger);

        telemetry.update();
    }

    /**
     * Get battery voltage
     */
    private double getBatteryVoltage() {
        double voltage = 0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }
        return voltage;
    }
}