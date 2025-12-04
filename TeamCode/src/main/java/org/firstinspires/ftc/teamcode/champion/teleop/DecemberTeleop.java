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
public class DecemberTeleop extends LinearOpMode {

    // Ramp angle increment (tunable via FTC Dashboard)
    public static double RAMP_INCREMENT_DEGREES = 10.0;

    // Turret angle increment for manual control
    public static double TURRET_INCREMENT_DEGREES = 5.0;

    // Turret alignment enable/disable
    public static boolean ENABLE_AUTO_ALIGNMENT = true;

    // Target AprilTag ID for turret alignment
    public static int TURRET_TARGET_TAG_ID = 20;

    // Shooter target RPM (tunable via FTC Dashboard)
    public static double SHOOTER_TARGET_RPM = 3000.0;

    // Shooter RPM increment per button press
    public static double SHOOTER_RPM_INCREMENT = 50.0;

    // Shooter RPM limits
    public static double SHOOTER_MIN_RPM = 0.0;
    public static double SHOOTER_MAX_RPM = 6000.0;

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

    // Player 1
    private boolean RightBumper1 = false;
    private boolean LeftTrigger1 = false;
    private boolean LeftBumper1 = false;
    private boolean Y1 = false;
    private boolean A1 = false;
    private boolean B1 = false;
    private boolean X1 = false;
    private boolean DpadDown = false;

    //Player 2
    private boolean RightBumper2 = false;
    private boolean RightTrigger2 = false;
    private boolean LeftBumper2 = false;
    private boolean Y2 = false;
    private boolean A2 = false;
    private boolean X2 = false;

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
            // ========== REVERSE CONTROL ==========
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

            manualPowerToggle();

            // ========== TELEMETRY ==========
            displayTelemetry();
        }
    }

    private void initializeHardware() {

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

        // Set initial target RPM from Dashboard-tunable value
        if (shooter != null) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }
    }

    private void displayInitScreen() {//TODO UPDATE TELEMETRY
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
        telemetry.addLine("  X: Increase RPM +50");
        telemetry.addLine("  A: Decrease RPM -50");
        telemetry.addLine("  Y: SHOOT (reach target RPM)");
        telemetry.addLine("  LT: Manual power (overrides)");
        telemetry.addLine();
        telemetry.addLine("🔄 B: REVERSE (Intake/Transfer/Uptake)");
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
     * Toggles reverse mode for intake, transfer, and uptake
     */
    private void handleReverse() {
        float currentRT = gamepad1.right_trigger;
        if (currentRT > 0.1 && !RightTrigger2) {//TODO MAKE THE TRIGGER BE AND ACTIVE VOMIT BUTTON
            // Toggle reverse mode for all systems
            if (intake != null) {
                if (intake.reversed) {
                    intake.reversed = false;
                    if (intake.isActive()) intake.toggle(); // Turn off
                } else {
                    intake.reversed = true;
                    if (!intake.isActive()) intake.toggle(); // Turn on in reverse
                }
            }

            if (transfer != null) {
                if (transfer.reversed) {
                    transfer.reversed = false;
                    if (transfer.isActive()) transfer.toggle();
                } else {
                    transfer.reversed = true;
                    if (!transfer.isActive()) transfer.toggle();
                }
            }

            if (uptake != null) {
                if (uptake.reversed) {
                    uptake.reversed = false;
                    if (uptake.isActive()) uptake.toggle();
                } else {
                    uptake.reversed = true;
                    if (!uptake.isActive()) uptake.toggle();
                }
            }
        } else {
            //TODO SET NORMAL VALUES WHEN NOT PRESSING TRIGGER
        }
    }

    /**
     * Driving
     */
    private void handleDriveControls() {
        double rawDrive = -gamepad2.left_stick_y;
        double rawTurn = gamepad2.right_stick_x;

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

    private void manualPowerToggle() {
        if (gamepad1.a && !A2) {
            A2 = true;
            uptake.toggle();
        } else if (!gamepad1.a && A2) {
            A2 = false;
        }

        if (gamepad1.x && !X2) {
            X2 = true;
            intake.toggle();
        } else if (!gamepad1.x && X2) {
            X2 = false;
        }

        if (gamepad1.y && !Y2) {
            Y2 = true;
            transfer.toggle();
        } else if (!gamepad1.y && Y2) {
            Y2 = false;
        }

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
        double turretTurnVel = gamepad1.left_stick_x;
        if (turret == null) return;

        if (turretTurnVel > 0 || turretTurnVel < 0) {
            manualTurretOverride = true;
        }

        if (gamepad1.dpad_down) { //TODO DPAD DOWN IS OVERRIDE DISABLE
            manualTurretOverride = false;
        }

        if (turretTurnVel > 0) {
            turret.incrementAngle(turretTurnVel);
        } else {
            turret.decrementAngle(TURRET_INCREMENT_DEGREES);
        }

        // If in manual mode and turret reached target, allow auto-align to resume
        if (manualTurretOverride && turret.atTarget())//TODO CHECK IS AUTO ALLIGNMENT WORKS

        {
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

        // Y = Ramp up
        boolean currentY1 = gamepad1.y;
        if (currentY1 && !Y1) {
            ramp.incrementAngle(RAMP_INCREMENT_DEGREES);
        }
        Y1 = currentY1;

        // B = ramp down
        boolean currentB = gamepad1.b;
        if (currentB && !B1) {
            ramp.decrementAngle(RAMP_INCREMENT_DEGREES);
        }
        B1 = currentB;
    }

    /**
     * Handle system toggles
     */
    private void handleSystemToggles() {
        // Right Bumper gamepad 2 - Intake toggle
        boolean currentRightBumper = gamepad2.right_bumper;
        if (currentRightBumper && !RightBumper2) {
            if (intake != null) {
                intake.reversed = false;
                intake.toggle();
                intake.update();
            }
        }
        RightBumper2 = currentRightBumper;

        // Both left bumpers toggle normal uptake and transfer
        boolean currentLeftBumper1 = gamepad1.left_bumper;
        boolean currentLeftBumper2 = gamepad2.left_bumper;
        if ((currentLeftBumper1 && !LeftBumper2) || (currentLeftBumper2 && !LeftBumper2)) {
            if (transfer != null) {
                transfer.reversed = false;
                transfer.toggle();
                transfer.update();
            }

            if (uptake != null) {
                uptake.reversed = false;
                uptake.toggle();
                uptake.update();
            }
        }
        LeftBumper1 = currentLeftBumper1;
        LeftBumper2 = currentLeftBumper2;
    }

    /**
     * Handle shooter control - X/A for RPM adjustment, Y for shoot toggle
     */
    private void handleShooterControl() {//TODO SET FAR AND CLOSE VALUES FOR A AND X
        if (shooter == null) return;

        // X Button - Increase target RPM by 50
        boolean currentX = gamepad1.x;
        if (currentX && !X1) {
            SHOOTER_TARGET_RPM = Math.min(SHOOTER_TARGET_RPM + SHOOTER_RPM_INCREMENT, SHOOTER_MAX_RPM);
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }
        X1 = currentX;

        // A Button - Decrease target RPM by 50
        boolean currentA = gamepad1.a;
        if (currentA && !A1) {
            SHOOTER_TARGET_RPM = Math.max(SHOOTER_TARGET_RPM - SHOOTER_RPM_INCREMENT, SHOOTER_MIN_RPM);
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }
        A1 = currentA;

        // Y Button - Toggle shoot mode
        boolean currentR1Bumper = gamepad1.right_bumper;
        if (currentR1Bumper && !RightBumper1) {
            // Ensure target RPM is set before toggling shoot mode
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            shooter.toggleShoot();
        }
        RightBumper1 = currentR1Bumper;

        // Left Trigger - Manual control (overrides PID)
        double triggerValue = gamepad1.left_trigger;
        if (triggerValue > 0.1) {
            shooterManualMode = true;
            shooter.stopShooting();
            shooter.setPower(triggerValue);
        } else if (shooterManualMode && triggerValue <= 0.1) {
            shooterManualMode = false;
            shooter.setPower(0);
        }

        // Update shooter PID control
        shooter.update();
    }

    private void updateAllSystems() {
        if (turret != null) turret.update();
        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
    }

    private void displayTelemetry() {//TODO CLEAN TELEMETRY AND ADD MODE VIEWER
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
        if (shooter != null) {
            telemetry.addLine();
            telemetry.addLine("═══ SHOOTER ═══");
            String shootMode = shooter.isShootMode() ? "🟢 SHOOTING" : "⚫ IDLE";
            if (shooterManualMode) shootMode = "🟡 MANUAL";
            telemetry.addData("Mode", shootMode);
            telemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
            telemetry.addData("RPM Error", "%.0f", shooter.getRPMError());
            telemetry.addData("At Target", shooter.isAtTargetRPM() ? "✓ YES" : "✗ NO");
            telemetry.addData("Power", "%.2f", shooter.getCurrentPower());
        }

        // Systems status
        telemetry.addLine();
        telemetry.addLine("═══ SYSTEMS ═══");
        telemetry.addData("Intake (RB)", "%s", (intake != null && intake.isActive()) ? "🟢 ON" : "⚫ OFF");
        telemetry.addData("Transfer (RT)", "%s", (transfer != null && transfer.isActive()) ? "🟢 ON" : "⚫ OFF");
        telemetry.addData("Uptake (LB)", "%s", (uptake != null && uptake.isActive()) ? "🟢 ON" : "⚫ OFF");

        telemetry.addLine();
        telemetry.addLine("DPAD L/R=Turret | U/D=Ramp | X/A=RPM±50 | Y=Shoot");

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