package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Full Robot Control", group="TeleOp")
public class RobotControl extends LinearOpMode {

    // ========== HARDWARE ==========
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private CRServo uptakeServo;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor;

    // ========== TURRET TRACKING ==========
    private ElapsedTime runtime = new ElapsedTime();
    private double lastPosition = 0;
    private double currentPosition = 0;
    private double velocity = 0;
    private double lastTime = 0;

    // ========== TURRET POSITION CONTROL ==========
    private boolean positionMode = false;
    private double targetPosition = 0;

    // ========== TOGGLE STATES ==========
    private boolean intakeActive = false;
    private boolean transferActive = false;
    private boolean uptakeActive = false;

    // Button debouncing
    private boolean lastRightBumper = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftBumper = false;

    // ========== TURRET CONSTANTS ==========
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;
    private static final double GEAR_RATIO = 2.5; // 2.5 servo degrees = 1 turret degree

    // ========== FTC DASHBOARD TUNABLE - TURRET ==========
    public static double TURRET_SERVO_POWER = 0.5;
    public static double TURRET_POSITION_TOLERANCE = 2.0;
    public static double TURRET_MANUAL_SPEED = 0.6;
    public static double TURRET_JOYSTICK_DEADBAND = 0.1;

    // Turret limits (in turret degrees)
    public static double MIN_TURRET_ANGLE = 0.0;
    public static double MAX_TURRET_ANGLE = 144.0;
    public static boolean ENABLE_TURRET_LIMITS = false;

    // Turret preset positions (in turret degrees)
    public static double TURRET_FRONT = 0.0;
    public static double TURRET_LEFT = 36.0;
    public static double TURRET_BACK = 72.0;
    public static double TURRET_RIGHT = 108.0;

    // ========== FTC DASHBOARD TUNABLE - MOTORS ==========
    public static double INTAKE_POWER = -1.0;
    public static double TRANSFER_POWER = -1.0;
    public static double UPTAKE_POWER = -1.0;  // Reversed as requested
    public static double SHOOTER_POWER = 1.0;

    // Motor direction settings
    public static boolean REVERSE_INTAKE = false;
    public static boolean REVERSE_TRANSFER = false;
    public static boolean REVERSE_SHOOTER = false;

    @Override
    public void runOpMode() {
        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize hardware
        try {
            turretServo = hardwareMap.get(CRServo.class, "turret");
            turretEncoder = hardwareMap.get(AnalogInput.class, "turret_analog");
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

            // Set motor modes
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set zero power behavior
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("✓ Status", "All hardware initialized");
        } catch (Exception e) {
            telemetry.addData("✗ ERROR", "Hardware initialization failed!");
            telemetry.addData("Details", e.getMessage());
            telemetry.update();
        }

        displayInitializationScreen();
        telemetry.update();

        waitForStart();
        runtime.reset();
        lastTime = runtime.seconds();

        while (opModeIsActive()) {
            // ========== UPDATE TURRET POSITION ==========
            updateTurretPosition();

            // ========== HANDLE ALL CONTROLS ==========
            handleTurretControls();
            handleToggleControls();

            // ========== CALCULATE POWERS ==========
            double turretPower = calculateTurretPower();
            double uptakePower = uptakeActive ? UPTAKE_POWER : 0.0;
            double intakePower = intakeActive ? INTAKE_POWER : 0.0;
            double transferPower = transferActive ? TRANSFER_POWER : 0.0;
            double shooterPower = gamepad1.left_trigger * SHOOTER_POWER;

            // ========== APPLY MOTOR DIRECTIONS ==========
            if (REVERSE_INTAKE) intakePower = -intakePower;
            if (REVERSE_TRANSFER) transferPower = -transferPower;
            if (REVERSE_SHOOTER) shooterPower = -shooterPower;

            // ========== SET ALL POWERS ==========
            turretServo.setPower(turretPower);
            uptakeServo.setPower(uptakePower);
            intakeMotor.setPower(intakePower);
            transferMotor.setPower(transferPower);
            shooterMotor.setPower(shooterPower);

            // ========== TELEMETRY ==========
            displayTelemetry(turretPower, uptakePower, intakePower, transferPower, shooterPower);
        }
    }

    /**
     * Display initialization screen with all controls
     */
    private void displayInitializationScreen() {
        telemetry.addData("", "");
        telemetry.addData("TURRET", "");
        telemetry.addData("  Right Stick X", "Manual control");
        telemetry.addData("  D-Pad", "Preset positions");
        telemetry.addData("  X Button", "Emergency stop");
        telemetry.addData("", "");
        telemetry.addData("  Right Bumper", "Intake");
        telemetry.addData("  Right Trigger", "Transfer");
        telemetry.addData("  Left Bumper", "Uptake");
        telemetry.addData("", "");
        telemetry.addData("SHOOTER", "");
        telemetry.addData("Left Trigger", "Variable speed");
    }

    /**
     * Update turret position from encoder
     */
    private void updateTurretPosition() {
        double voltage = turretEncoder.getVoltage();
        double servoPosition = voltage * VOLTAGE_TO_DEGREES;
        currentPosition = servoPosition / GEAR_RATIO;

        // Calculate velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0) {
            double deltaPosition = currentPosition - lastPosition;

            // Handle wraparound
            if (deltaPosition > 180) {
                deltaPosition -= 360;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
            }

            velocity = deltaPosition / deltaTime;
        }

        lastPosition = currentPosition;
        lastTime = currentTime;
    }

    /**
     * Handle turret control inputs
     */
    private void handleTurretControls() {
        // D-Pad preset positions
        if (gamepad1.dpad_up) {
            setTargetPosition(TURRET_FRONT);
        } else if (gamepad1.dpad_left) {
            setTargetPosition(TURRET_LEFT);
        } else if (gamepad1.dpad_down) {
            setTargetPosition(TURRET_BACK);
        } else if (gamepad1.dpad_right) {
            setTargetPosition(TURRET_RIGHT);
        }

        // Emergency stop
        if (gamepad1.x) {
            positionMode = false;
        }

        // Manual control overrides position mode
        double manualInput = gamepad1.right_stick_x;
        if (Math.abs(manualInput) > TURRET_JOYSTICK_DEADBAND) {
            positionMode = false;
        }
    }

    /**
     * Handle toggle controls for intake, transfer, and uptake
     */
    private void handleToggleControls() {
        // Right Bumper - Intake toggle
        boolean currentRightBumper = gamepad1.right_bumper;
        if (currentRightBumper && !lastRightBumper) {
            intakeActive = !intakeActive;
        }
        lastRightBumper = currentRightBumper;

        // Right Trigger - Transfer toggle (using threshold)
        boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
        if (currentRightTrigger && !lastRightTrigger) {
            transferActive = !transferActive;
        }
        lastRightTrigger = currentRightTrigger;

        // Left Bumper - Uptake toggle
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumper) {
            uptakeActive = !uptakeActive;
        }
        lastLeftBumper = currentLeftBumper;
    }

    /**
     * Calculate turret power based on mode
     */
    private double calculateTurretPower() {
        double power = 0;
        double manualInput = gamepad1.right_stick_x;

        // Manual control has priority
        if (Math.abs(manualInput) > TURRET_JOYSTICK_DEADBAND) {
            power = manualInput * TURRET_MANUAL_SPEED;

            // Apply soft limits if enabled
            if (ENABLE_TURRET_LIMITS) {
                if (currentPosition <= MIN_TURRET_ANGLE && power < 0) {
                    power = 0;
                } else if (currentPosition >= MAX_TURRET_ANGLE && power > 0) {
                    power = 0;
                }
            }
        } else if (positionMode) {
            // Position control mode
            double error = targetPosition - currentPosition;

            // Normalize error to [-180, 180] for shortest path
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Check if we've reached target
            if (Math.abs(error) < TURRET_POSITION_TOLERANCE) {
                power = 0;
                positionMode = false;
            } else {
                // Simple proportional control
                power = (error > 0) ? TURRET_SERVO_POWER : -TURRET_SERVO_POWER;
            }
        }

        return power;
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(double turretPower, double uptakePower, double intakePower,
                                  double transferPower, double shooterPower) {
        double voltage = turretEncoder.getVoltage();
        double servoPosition = voltage * VOLTAGE_TO_DEGREES;
        double batteryVoltage = getBatteryVoltage();

        // Header
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("", "");

        // Turret status
        telemetry.addData("TURRET", "");
        telemetry.addData("Mode", positionMode ? "AUTO" : "MANUAL");
        telemetry.addData("Position", "%.2f° (turret)", currentPosition);
        telemetry.addData("Raw Voltage", "%.3f V", voltage);
        telemetry.addData("Power", "%.3f", turretPower);

        if (positionMode) {
            double error = targetPosition - currentPosition;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            telemetry.addData("Target", "%.2f°", targetPosition);
            telemetry.addData("Error", "%.2f°", error);
        }
        telemetry.addData("Shooter", "%.2f (LT: %.2f)",
                shooterPower, gamepad1.left_trigger);

        String activePreset = "";
        if (isNear(currentPosition, TURRET_FRONT)) activePreset = "FRONT ✓";
        else if (isNear(currentPosition, TURRET_LEFT)) activePreset = "LEFT ✓";
        else if (isNear(currentPosition, TURRET_BACK)) activePreset = "BACK ✓";
        else if (isNear(currentPosition, TURRET_RIGHT)) activePreset = "RIGHT ✓";

        if (!activePreset.isEmpty()) {
            telemetry.addData("At Position", activePreset);
        } else {
            telemetry.addData("Presets", "F:%.0f° L:%.0f° B:%.0f° R:%.0f°",
                    TURRET_FRONT, TURRET_LEFT, TURRET_BACK, TURRET_RIGHT);
        }

        // Quick controls reminder
        telemetry.addData("", "");
        telemetry.addData("Controls", "RB=Intake | RT=Transfer | LB=Uptake | LT=Shooter");

        telemetry.update();
    }

    /**
     * Set target position and enable position mode
     */
    private void setTargetPosition(double target) {
        targetPosition = target;
        positionMode = true;
    }

    /**
     * Check if current position is near target
     */
    private boolean isNear(double current, double target) {
        double error = Math.abs(target - current);
        if (error > 180) error = 360 - error;
        return error < TURRET_POSITION_TOLERANCE * 2;
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