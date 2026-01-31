package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Velocity PIDF Tuning OpMode
 *
 * USE WITH FTC DASHBOARD (192.168.43.1:8080/dash)
 *
 * === TUNING STEPS ===
 *
 * STEP 1: FIND kF (Feedforward)
 * - Set all gains to 0
 * - Set TEST_MODE = FEEDFORWARD_TUNING
 * - Run at TARGET_VELOCITY
 * - Adjust TUNE_F until actual velocity ≈ 80-90% of target
 * - Formula: kF = (power needed) / (target velocity) * MAX_TICKS_PER_SEC
 *
 * STEP 2: TUNE kP (Proportional)
 * - Set TEST_MODE = STEP_RESPONSE
 * - Start with small kP (e.g., 10)
 * - Increase until slight overshoot/oscillation
 * - Back off 20-30%
 *
 * STEP 3: TUNE kD (Derivative)
 * - If oscillating, add kD
 * - Start with kD = kP * 0.1
 * - Increase until oscillation stops
 * - Don't go too high (causes jitter)
 *
 * STEP 4: TUNE kI (Integral) - OPTIONAL
 * - Only if steady-state error persists
 * - Start very small (e.g., 0.1)
 * - Increase slowly until error eliminated
 *
 * === CONTROLS ===
 *
 * A - Start/stop test
 * B - Switch test mode
 * X - Test LEFT side only
 * Y - Test RIGHT side only
 * Left Bumper - Test BOTH sides
 * Right Bumper - Apply current TUNE values to motors
 *
 * Dpad Up/Down - Adjust target velocity
 * Left Stick Y - Manual velocity control (when test stopped)
 */
@Config
@TeleOp(name = "Velocity PIDF Tuner", group = "Tuning")
public class VelocityPIDF extends LinearOpMode {

    // === TUNABLE PARAMETERS (Adjust in Dashboard) ===

    // PIDF Values to tune
    public static double TUNE_P = 0.0;
    public static double TUNE_I = 0.0;
    public static double TUNE_D = 0.5;
    public static double TUNE_F = 12.0;

    // Test parameters
    public static double TARGET_VELOCITY = 2000.0;  // ticks/sec
    public static double VELOCITY_INCREMENT = 200.0;

    // Motor specs (goBILDA 312 RPM)
    public static double TICKS_PER_REV = 751.8;
    public static double MAX_RPM = 312.0;
    public static double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;  // ~3909

    // Test modes
    public enum TestMode {
        STEP_RESPONSE,      // Sudden velocity change (0 -> target)
        FEEDFORWARD_TUNING, // Constant velocity for kF tuning
        OSCILLATION_TEST,   // Alternating velocities
        RAMP_TEST           // Gradual velocity increase
    }
    public static TestMode TEST_MODE = TestMode.STEP_RESPONSE;

    // Which side to test
    public enum TestSide {
        LEFT,
        RIGHT,
        BOTH
    }
    public static TestSide TEST_SIDE = TestSide.BOTH;

    // Step response parameters
    public static double STEP_DURATION_SEC = 2.0;
    public static double REST_DURATION_SEC = 1.0;

    // Oscillation test parameters
    public static double OSCILLATION_PERIOD_SEC = 1.0;
    public static double OSCILLATION_LOW_VEL = 1000.0;
    public static double OSCILLATION_HIGH_VEL = 3000.0;

    // === HARDWARE ===
    private DcMotorEx lf, rf, lb, rb;
    private FtcDashboard dashboard;

    // === STATE ===
    private boolean testRunning = false;
    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();

    // Button state tracking
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private boolean lastLB = false, lastRB = false;
    private boolean lastDpadUp = false, lastDpadDown = false;

    // Data logging
    private double lastLeftVelocity = 0;
    private double lastRightVelocity = 0;
    private double lastTargetVelocity = 0;
    private double leftError = 0;
    private double rightError = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Setup dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== VELOCITY PIDF TUNER ===");
        telemetry.addLine("Connect to FTC Dashboard: 192.168.43.1:8080/dash");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // Apply initial PIDF values
        applyPIDF();

        while (opModeIsActive()) {
            loopTimer.reset();

            handleControls();

            if (testRunning) {
                runTest();
            } else {
                // Manual control when test not running
                double manualPower = -gamepad1.left_stick_y;
                if (Math.abs(manualPower) > 0.1) {
                    double manualVelocity = manualPower * MAX_TICKS_PER_SEC;
                    setVelocity(manualVelocity, manualVelocity);
                } else {
                    setVelocity(0, 0);
                }
            }

            // Read velocities
            updateVelocityReadings();

            // Send data to dashboard
            sendDashboardData();

            // Display telemetry
            displayTelemetry();
        }

        // Stop motors
        stopMotors();
    }

    private void initHardware() {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Set directions
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake mode
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Velocity mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void applyPIDF() {
        lf.setVelocityPIDFCoefficients(TUNE_P, TUNE_I, TUNE_D, TUNE_F);
        rf.setVelocityPIDFCoefficients(TUNE_P, TUNE_I, TUNE_D, TUNE_F);
        lb.setVelocityPIDFCoefficients(TUNE_P, TUNE_I, TUNE_D, TUNE_F);
        rb.setVelocityPIDFCoefficients(TUNE_P, TUNE_I, TUNE_D, TUNE_F);
    }

    private void handleControls() {
        // A - Start/stop test
        if (gamepad1.a && !lastA) {
            testRunning = !testRunning;
            if (testRunning) {
                testTimer.reset();
                applyPIDF();  // Apply latest values when starting
            } else {
                stopMotors();
            }
        }

        // B - Cycle test mode
        if (gamepad1.b && !lastB) {
            TestMode[] modes = TestMode.values();
            int nextIndex = (TEST_MODE.ordinal() + 1) % modes.length;
            TEST_MODE = modes[nextIndex];
        }

        // X - Test left only
        if (gamepad1.x && !lastX) {
            TEST_SIDE = TestSide.LEFT;
        }

        // Y - Test right only
        if (gamepad1.y && !lastY) {
            TEST_SIDE = TestSide.RIGHT;
        }

        // Left Bumper - Test both
        if (gamepad1.left_bumper && !lastLB) {
            TEST_SIDE = TestSide.BOTH;
        }

        // Right Bumper - Apply PIDF values
        if (gamepad1.right_bumper && !lastRB) {
            applyPIDF();
        }

        // Dpad - Adjust target velocity
        if (gamepad1.dpad_up && !lastDpadUp) {
            TARGET_VELOCITY = Math.min(MAX_TICKS_PER_SEC, TARGET_VELOCITY + VELOCITY_INCREMENT);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - VELOCITY_INCREMENT);
        }

        // Update button states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    private void runTest() {
        double targetVel = 0;
        double elapsed = testTimer.seconds();

        switch (TEST_MODE) {
            case STEP_RESPONSE:
                // Alternate between 0 and target velocity
                double cycleDuration = STEP_DURATION_SEC + REST_DURATION_SEC;
                double cyclePosition = elapsed % cycleDuration;
                targetVel = (cyclePosition < STEP_DURATION_SEC) ? TARGET_VELOCITY : 0;
                break;

            case FEEDFORWARD_TUNING:
                // Constant velocity
                targetVel = TARGET_VELOCITY;
                break;

            case OSCILLATION_TEST:
                // Alternate between low and high velocity
                double oscPosition = elapsed % OSCILLATION_PERIOD_SEC;
                targetVel = (oscPosition < OSCILLATION_PERIOD_SEC / 2) ?
                        OSCILLATION_HIGH_VEL : OSCILLATION_LOW_VEL;
                break;

            case RAMP_TEST:
                // Gradual increase over 5 seconds, then reset
                double rampPosition = elapsed % 5.0;
                targetVel = (rampPosition / 5.0) * TARGET_VELOCITY;
                break;
        }

        lastTargetVelocity = targetVel;

        // Apply to selected side(s)
        double leftVel = (TEST_SIDE == TestSide.RIGHT) ? 0 : targetVel;
        double rightVel = (TEST_SIDE == TestSide.LEFT) ? 0 : targetVel;

        setVelocity(leftVel, rightVel);
    }

    private void setVelocity(double leftVel, double rightVel) {
        lf.setVelocity(leftVel);
        lb.setVelocity(leftVel);
        rf.setVelocity(rightVel);
        rb.setVelocity(rightVel);
    }

    private void stopMotors() {
        setVelocity(0, 0);
    }

    private void updateVelocityReadings() {
        // Read from front motors (you can change to back if needed)
        lastLeftVelocity = lf.getVelocity();
        lastRightVelocity = rf.getVelocity();

        // Calculate errors
        leftError = lastTargetVelocity - lastLeftVelocity;
        rightError = lastTargetVelocity - lastRightVelocity;
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Target vs Actual (main graph)
        packet.put("Velocity/Target", lastTargetVelocity);
        packet.put("Velocity/Left Actual", lastLeftVelocity);
        packet.put("Velocity/Right Actual", lastRightVelocity);

        // Errors
        packet.put("Error/Left", leftError);
        packet.put("Error/Right", rightError);

        // Percentages
        if (lastTargetVelocity > 0) {
            packet.put("Percent/Left", (lastLeftVelocity / lastTargetVelocity) * 100);
            packet.put("Percent/Right", (lastRightVelocity / lastTargetVelocity) * 100);
        }

        // PIDF values being used
        packet.put("PIDF/P", TUNE_P);
        packet.put("PIDF/I", TUNE_I);
        packet.put("PIDF/D", TUNE_D);
        packet.put("PIDF/F", TUNE_F);

        // Motor powers
        packet.put("Power/LF", lf.getPower() * 100);
        packet.put("Power/RF", rf.getPower() * 100);

        dashboard.sendTelemetryPacket(packet);
    }

    private void displayTelemetry() {
        telemetry.addLine("=== VELOCITY PIDF TUNER ===");
        telemetry.addLine();

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A", "Start/Stop Test: " + (testRunning ? "RUNNING" : "STOPPED"));
        telemetry.addData("B", "Test Mode: " + TEST_MODE);
        telemetry.addData("X/Y/LB", "Side: " + TEST_SIDE);
        telemetry.addData("RB", "Apply PIDF Values");
        telemetry.addData("Dpad", "Adjust Target Velocity");
        telemetry.addLine();

        telemetry.addLine("--- PIDF VALUES ---");
        telemetry.addData("P", "%.2f", TUNE_P);
        telemetry.addData("I", "%.4f", TUNE_I);
        telemetry.addData("D", "%.2f", TUNE_D);
        telemetry.addData("F", "%.2f", TUNE_F);
        telemetry.addLine();

        telemetry.addLine("--- TARGET ---");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", TARGET_VELOCITY);
        telemetry.addData("Max Possible", "%.0f ticks/sec", MAX_TICKS_PER_SEC);
        telemetry.addData("Target %", "%.0f%%", (TARGET_VELOCITY / MAX_TICKS_PER_SEC) * 100);
        telemetry.addLine();

        telemetry.addLine("--- ACTUAL ---");
        telemetry.addData("Left Velocity", "%.0f ticks/sec", lastLeftVelocity);
        telemetry.addData("Right Velocity", "%.0f ticks/sec", lastRightVelocity);
        telemetry.addLine();

        telemetry.addLine("--- ERROR ---");
        telemetry.addData("Left Error", "%.0f (%.1f%%)", leftError,
                lastTargetVelocity > 0 ? (leftError / lastTargetVelocity) * 100 : 0);
        telemetry.addData("Right Error", "%.0f (%.1f%%)", rightError,
                lastTargetVelocity > 0 ? (rightError / lastTargetVelocity) * 100 : 0);
        telemetry.addLine();

        telemetry.addLine("--- MOTOR POWER ---");
        telemetry.addData("LF/LB", "%.2f / %.2f", lf.getPower(), lb.getPower());
        telemetry.addData("RF/RB", "%.2f / %.2f", rf.getPower(), rb.getPower());
        telemetry.addLine();

        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());

        telemetry.update();
    }
}