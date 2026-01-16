package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * UPTAKE SWITCH TESTER
 *
 * Tests the uptake ball detection switch to find the correct threshold.
 *
 * HOW TO USE:
 * 1. Run this OpMode
 * 2. Watch the voltage reading with NO ball present
 * 3. Put a ball against the switch
 * 4. Watch the voltage reading WITH ball present
 * 5. The threshold should be BETWEEN these two values
 *
 * CONTROLS:
 *   A = Mark current voltage as "NO BALL" reading
 *   B = Mark current voltage as "BALL DETECTED" reading
 *   Y = Calculate recommended threshold
 *
 * COMMON SWITCH BEHAVIORS:
 *   Type 1: Voltage HIGH when no ball, LOW when ball (most common)
 *   Type 2: Voltage LOW when no ball, HIGH when ball
 *
 * The code will auto-detect which type you have!
 */
@TeleOp(name = "!! Uptake Switch Tester !!", group = "Test")
public class UptakeSwitch extends LinearOpMode {

    private AnalogInput uptakeSwitch;

    // Recorded values
    private double noBallVoltage = -1;
    private double ballDetectedVoltage = -1;
    private double recommendedThreshold = -1;
    private String switchType = "UNKNOWN";

    // Button states
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;

    // Voltage tracking
    private double minVoltage = 999;
    private double maxVoltage = -999;
    private double avgVoltage = 0;
    private int sampleCount = 0;

    // History for stability check
    private double[] voltageHistory = new double[50];
    private int historyIndex = 0;

    @Override
    public void runOpMode() {

        // Try to find the switch
        boolean switchFound = false;

        // Try different possible names
        String[] possibleNames = {"uptakeSwitch", "switch", "ball_switch", "uptake_sensor", "ballSwitch"};

        for (String name : possibleNames) {
            try {
                uptakeSwitch = hardwareMap.get(AnalogInput.class, name);
                telemetry.addLine("SUCCESS: Found switch as '" + name + "'");
                switchFound = true;
                break;
            } catch (Exception e) {
                // Try next name
            }
        }

        if (!switchFound) {
            telemetry.addLine("!!! ERROR: Switch not found !!!");
            telemetry.addLine();
            telemetry.addLine("Tried these names:");
            for (String name : possibleNames) {
                telemetry.addLine("  - " + name);
            }
            telemetry.addLine();
            telemetry.addLine("Check your hardwareMap configuration!");
            telemetry.update();

            waitForStart();
            while (opModeIsActive()) {
                telemetry.addLine("!!! SWITCH NOT FOUND !!!");
                telemetry.addLine("Check configuration and restart");
                telemetry.update();
                sleep(100);
            }
            return;
        }

        telemetry.addLine();
        telemetry.addLine("=== UPTAKE SWITCH TESTER ===");
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Start with NO ball near switch");
        telemetry.addLine("2. Press A to record 'no ball' voltage");
        telemetry.addLine("3. Put ball against switch");
        telemetry.addLine("4. Press B to record 'ball' voltage");
        telemetry.addLine("5. Press Y to calculate threshold");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            // Read current voltage
            double currentVoltage = uptakeSwitch.getVoltage();

            // Update statistics
            minVoltage = Math.min(minVoltage, currentVoltage);
            maxVoltage = Math.max(maxVoltage, currentVoltage);
            sampleCount++;
            avgVoltage = avgVoltage + (currentVoltage - avgVoltage) / sampleCount;

            // Update history
            voltageHistory[historyIndex] = currentVoltage;
            historyIndex = (historyIndex + 1) % voltageHistory.length;

            // Calculate stability (standard deviation of recent samples)
            double stability = calculateStability();

            // Handle A button - record NO BALL voltage
            if (gamepad1.a && !lastA) {
                noBallVoltage = currentVoltage;
            }
            lastA = gamepad1.a;

            // Handle B button - record BALL DETECTED voltage
            if (gamepad1.b && !lastB) {
                ballDetectedVoltage = currentVoltage;
            }
            lastB = gamepad1.b;

            // Handle Y button - calculate threshold
            if (gamepad1.y && !lastY) {
                if (noBallVoltage >= 0 && ballDetectedVoltage >= 0) {
                    // Threshold is midpoint between the two readings
                    recommendedThreshold = (noBallVoltage + ballDetectedVoltage) / 2.0;

                    // Determine switch type
                    if (noBallVoltage > ballDetectedVoltage) {
                        switchType = "Type 1: HIGH→LOW (voltage drops when ball detected)";
                    } else {
                        switchType = "Type 2: LOW→HIGH (voltage rises when ball detected)";
                    }
                }
            }
            lastY = gamepad1.y;

            // Determine current ball state if threshold is set
            String ballState = "---";
            if (recommendedThreshold >= 0) {
                if (noBallVoltage > ballDetectedVoltage) {
                    // Type 1: voltage drops when ball detected
                    ballState = (currentVoltage < recommendedThreshold) ? ">>> BALL DETECTED <<<" : "no ball";
                } else {
                    // Type 2: voltage rises when ball detected
                    ballState = (currentVoltage > recommendedThreshold) ? ">>> BALL DETECTED <<<" : "no ball";
                }
            }

            // Telemetry
            telemetry.addLine("=== UPTAKE SWITCH TESTER ===");
            telemetry.addLine();

            // Live readings
            telemetry.addLine("--- LIVE READINGS ---");
            telemetry.addData("Current Voltage", "%.3f V", currentVoltage);
            telemetry.addData("Min / Max", "%.3f / %.3f V", minVoltage, maxVoltage);
            telemetry.addData("Average", "%.3f V", avgVoltage);
            telemetry.addData("Stability", stability < 0.05 ? "STABLE" : "FLUCTUATING (%.3f)");
            telemetry.addLine();

            // Recorded values
            telemetry.addLine("--- RECORDED VALUES ---");
            if (noBallVoltage >= 0) {
                telemetry.addData("No Ball Voltage (A)", "%.3f V", noBallVoltage);
            } else {
                telemetry.addLine("No Ball Voltage: [Press A to record]");
            }

            if (ballDetectedVoltage >= 0) {
                telemetry.addData("Ball Detected Voltage (B)", "%.3f V", ballDetectedVoltage);
            } else {
                telemetry.addLine("Ball Voltage: [Press B to record]");
            }
            telemetry.addLine();

            // Results
            if (recommendedThreshold >= 0) {
                telemetry.addLine("--- RESULTS ---");
                telemetry.addData("RECOMMENDED THRESHOLD", "%.3f V", recommendedThreshold);
                telemetry.addLine(switchType);
                telemetry.addLine();
                telemetry.addData("BALL STATE", ballState);
                telemetry.addLine();
                telemetry.addLine("--- CODE TO USE ---");
                telemetry.addData("UPTAKE_SWITCH_THRESHOLD", "%.2f", recommendedThreshold);
                if (noBallVoltage > ballDetectedVoltage) {
                    telemetry.addLine("Detection: voltage < threshold");
                } else {
                    telemetry.addLine("Detection: voltage > threshold");
                }
            } else {
                telemetry.addLine("--- RESULTS ---");
                telemetry.addLine("[Press Y after recording both values]");
            }

            telemetry.addLine();
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("A = Record NO BALL voltage");
            telemetry.addLine("B = Record BALL DETECTED voltage");
            telemetry.addLine("Y = Calculate threshold");

            // Visual voltage bar
            telemetry.addLine();
            telemetry.addLine("--- VOLTAGE BAR (0-3.3V) ---");
            int barLength = (int)((currentVoltage / 3.3) * 30);
            StringBuilder bar = new StringBuilder("[");
            for (int i = 0; i < 30; i++) {
                if (i < barLength) {
                    bar.append("█");
                } else {
                    bar.append("░");
                }
            }
            bar.append("]");
            telemetry.addLine(bar.toString());

            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Calculate stability (standard deviation) of recent voltage readings
     */
    private double calculateStability() {
        if (sampleCount < voltageHistory.length) {
            return 0;
        }

        double sum = 0;
        for (double v : voltageHistory) {
            sum += v;
        }
        double mean = sum / voltageHistory.length;

        double varianceSum = 0;
        for (double v : voltageHistory) {
            varianceSum += (v - mean) * (v - mean);
        }

        return Math.sqrt(varianceSum / voltageHistory.length);
    }
}