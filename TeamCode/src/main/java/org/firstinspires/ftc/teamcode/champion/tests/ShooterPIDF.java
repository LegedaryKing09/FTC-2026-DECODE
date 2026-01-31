package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter PIDF Tuning Test
 *
 * CONTROLS:
 * - A: Start/Stop shooter
 * - X: Switch to LOW preset (3800 RPM)
 * - Y: Switch to HIGH preset (4600 RPM)
 * - DPAD UP/DOWN: Fine adjust target RPM (+/- 50)
 * - B: Reset timing stats
 *
 * DASHBOARD:
 * - Graphs RPM, Target, Error, Power over time
 * - Tune kP, kI, kD, kF in real-time
 *
 * TUNING PROCESS:
 * 1. Set kI=0, kD=0, kF=0
 * 2. Find kF: Set target RPM, manually find power that maintains it
 *    kF ≈ power / targetRPM (e.g., 0.8 / 4000 = 0.0002)
 * 3. Tune kP: Increase until oscillates, back off 30%
 * 4. Tune kD: Add to stop oscillation (usually kD = kP * 0.01 to 0.1)
 * 5. Tune kI: Only if steady-state error persists (usually 0)
 */
@Config
@TeleOp(name = "Shooter PIDF Tuning", group = "Test")
public class ShooterPIDF extends OpMode {

    // ========== TUNABLE PARAMETERS (Dashboard) ==========
    public static double kP = 0;      // Start here after finding kF
    public static double kI = 0.0;         // Usually 0 for flywheels
    public static double kD = 0;     // Small amount for damping
    public static double kF = 0;     // FIND THIS FIRST! power/RPM

    public static double LOW_RPM = 3800.0;
    public static double HIGH_RPM = 4600.0;
    public static double RPM_INCREMENT = 50.0;

    public static double INTEGRAL_LIMIT = 0.5;  // Anti-windup
    public static double MAX_POWER = 1.0;

    // Tolerance for "at target" timing
    public static double RPM_TOLERANCE = 100.0;

    // Encoder ticks per revolution (check your motor spec)
    public static double TICKS_PER_REV = 28.0;

    // ========== MOTOR ==========
    private DcMotor shooterMotor;

    // ========== STATE ==========
    private boolean isRunning = false;
    private double targetRPM = HIGH_RPM;
    private double currentRPM = 0;
    private double currentPower = 0;

    // ========== PID STATE ==========
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastPidTime = 0;

    // ========== TIMING STATS ==========
    private ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean timing = false;
    private double lastSpinUpTime = 0;
    private double lastSwitchTime = 0;
    private double bestSpinUpTime = 999;
    private double worstSpinUpTime = 0;
    private int spinUpCount = 0;
    private double totalSpinUpTime = 0;

    // ========== DASHBOARD ==========
    private FtcDashboard dashboard;

    // ========== BUTTON DEBOUNCE ==========
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // Initialize motor
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("Shooter", "FAILED: " + e.getMessage());
        }

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        pidTimer.reset();
        lastPidTime = pidTimer.seconds();

        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
        telemetry.addLine("A: Start/Stop");
        telemetry.addLine("X: LOW preset (3800)");
        telemetry.addLine("Y: HIGH preset (4600)");
        telemetry.addLine("DPAD: Fine adjust");
        telemetry.addLine("B: Reset stats");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== READ CONTROLS ==========
        handleControls();

        // ========== UPDATE RPM ==========
        updateRPM();

        // ========== RUN PIDF ==========
        if (isRunning) {
            runPIDF();
        } else {
            shooterMotor.setPower(0);
            currentPower = 0;
            integralSum = 0;
            lastError = 0;
        }

        // ========== UPDATE TIMING ==========
        updateTiming();

        // ========== SEND TO DASHBOARD ==========
        sendDashboardData();

        // ========== TELEMETRY ==========
        updateTelemetry();
    }

    private void handleControls() {
        // A - Start/Stop
        if (gamepad1.a && !lastA) {
            isRunning = !isRunning;
            if (isRunning) {
                // Starting - begin timing
                spinUpTimer.reset();
                timing = true;
                integralSum = 0;
                lastError = 0;
            } else {
                timing = false;
            }
        }
        lastA = gamepad1.a;

        // X - LOW preset
        if (gamepad1.x && !lastX) {
            double oldTarget = targetRPM;
            targetRPM = LOW_RPM;
            if (isRunning && Math.abs(oldTarget - targetRPM) > 100) {
                // Switching presets - time the transition
                spinUpTimer.reset();
                timing = true;
                // Reset integral for large changes
                integralSum = 0;
            }
        }
        lastX = gamepad1.x;

        // Y - HIGH preset
        if (gamepad1.y && !lastY) {
            double oldTarget = targetRPM;
            targetRPM = HIGH_RPM;
            if (isRunning && Math.abs(oldTarget - targetRPM) > 100) {
                spinUpTimer.reset();
                timing = true;
                integralSum = 0;
            }
        }
        lastY = gamepad1.y;

        // DPAD UP - Increment
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetRPM = Math.min(targetRPM + RPM_INCREMENT, 6000);
        }
        lastDpadUp = gamepad1.dpad_up;

        // DPAD DOWN - Decrement
        if (gamepad1.dpad_down && !lastDpadDown) {
            targetRPM = Math.max(targetRPM - RPM_INCREMENT, 0);
        }
        lastDpadDown = gamepad1.dpad_down;

        // B - Reset stats
        if (gamepad1.b && !lastB) {
            resetStats();
        }
        lastB = gamepad1.b;
    }

    private void updateRPM() {
        if (shooterMotor != null) {
            // Get velocity in ticks per second
            double tps = ((com.qualcomm.robotcore.hardware.DcMotorEx) shooterMotor).getVelocity();
            // Convert to RPM
            currentRPM = Math.abs((tps / TICKS_PER_REV) * 60.0);
        }
    }

    private void runPIDF() {
        double now = pidTimer.seconds();
        double dt = now - lastPidTime;

        if (dt < 0.005) return; // 200Hz max

        double error = targetRPM - currentRPM;

        // ========== FEEDFORWARD ==========
        // This is the KEY for flywheels!
        // Predicts the base power needed for target RPM
        double ffTerm = kF * targetRPM;

        // ========== PROPORTIONAL ==========
        double pTerm = kP * error;

        // ========== INTEGRAL ==========
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        double iTerm = kI * integralSum;

        // ========== DERIVATIVE ==========
        double dTerm = kD * (error - lastError) / dt;

        // ========== TOTAL OUTPUT ==========
        double output = ffTerm + pTerm + iTerm + dTerm;

        // Clamp to valid range (0 to MAX for flywheel)
        output = Math.max(0, Math.min(MAX_POWER, output));

        shooterMotor.setPower(output);
        currentPower = output;

        lastError = error;
        lastPidTime = now;
    }

    private void updateTiming() {
        if (!timing) return;

        double error = Math.abs(targetRPM - currentRPM);

        if (error <= RPM_TOLERANCE) {
            // Reached target!
            double timeToReach = spinUpTimer.seconds();
            timing = false;

            lastSpinUpTime = timeToReach;
            spinUpCount++;
            totalSpinUpTime += timeToReach;

            if (timeToReach < bestSpinUpTime) {
                bestSpinUpTime = timeToReach;
            }
            if (timeToReach > worstSpinUpTime) {
                worstSpinUpTime = timeToReach;
            }
        }
    }

    private void resetStats() {
        bestSpinUpTime = 999;
        worstSpinUpTime = 0;
        spinUpCount = 0;
        totalSpinUpTime = 0;
        lastSpinUpTime = 0;
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Main graph values
        packet.put("Shooter/RPM", currentRPM);
        packet.put("Shooter/Target", targetRPM);
        packet.put("Shooter/Error", targetRPM - currentRPM);
        packet.put("Shooter/Power", currentPower * 1000); // Scale for visibility

        // PID terms (scaled for visibility)
        double error = targetRPM - currentRPM;
        packet.put("Shooter/FF", kF * targetRPM * 1000);
        packet.put("Shooter/P", kP * error * 1000);
        packet.put("Shooter/I", kI * integralSum * 1000);

        // Timing
        packet.put("Shooter/SpinUpTime", lastSpinUpTime);
        packet.put("Shooter/Running", isRunning ? 1 : 0);

        dashboard.sendTelemetryPacket(packet);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Running", isRunning ? "YES" : "NO");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Error", "%.0f", targetRPM - currentRPM);
        telemetry.addData("Power", "%.3f", currentPower);

        telemetry.addLine("");
        telemetry.addLine("=== PIDF GAINS ===");
        telemetry.addData("kF", "%.6f", kF);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.6f", kD);

        telemetry.addLine("");
        telemetry.addLine("=== TIMING STATS ===");
        telemetry.addData("Last Time", "%.3f sec", lastSpinUpTime);
        telemetry.addData("Best Time", "%.3f sec", bestSpinUpTime < 999 ? bestSpinUpTime : 0);
        telemetry.addData("Worst Time", "%.3f sec", worstSpinUpTime);
        telemetry.addData("Avg Time", "%.3f sec", spinUpCount > 0 ? totalSpinUpTime / spinUpCount : 0);
        telemetry.addData("Count", spinUpCount);

        if (timing) {
            telemetry.addLine("");
            telemetry.addData("TIMING", "%.3f sec...", spinUpTimer.seconds());
        }

        telemetry.addLine("");
        telemetry.addLine("=== PRESETS ===");
        telemetry.addData("X = LOW", "%.0f RPM", LOW_RPM);
        telemetry.addData("Y = HIGH", "%.0f RPM", HIGH_RPM);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
    }
}