package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ramp with Analog Test")
public class RampTest extends LinearOpMode {

    // Conversion constant for Axon servo
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;

    // Incremental control settings
    private static final double INCREMENT_POWER = 0.3;  // Power for small movements
    private static final long INCREMENT_TIME_MS = 100;   // How long to pulse (milliseconds)

    // Button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("=== RAMP ANALOG TEST ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Get hardware
        CRServo rampServo = null;
        AnalogInput rampAnalog = null;

        try {
            rampServo = hardwareMap.get(CRServo.class, "ramp");
            // REVERSE the servo direction since only -1 power works
            rampServo.setDirection(CRServo.Direction.REVERSE);
            telemetry.addLine("Ramp servo found");
        } catch (Exception e) {
            telemetry.addLine("Ramp servo NOT found!");
            telemetry.addLine("Error: " + e.getMessage());
        }

        try {
            rampAnalog = hardwareMap.get(AnalogInput.class, "ramp_analog");
            telemetry.addLine("Ramp analog found");
        } catch (Exception e) {
            telemetry.addLine("Ramp analog NOT found");
            telemetry.addLine("Will work without feedback");
        }

        telemetry.addLine();
        telemetry.addLine("DPAD UP: Small increment up");
        telemetry.addLine("DPAD DOWN: Small increment down");
        telemetry.addLine("DPAD LEFT: Continuous down");
        telemetry.addLine("DPAD RIGHT: Continuous up");
        telemetry.addLine("A: STOP");
        telemetry.addLine();
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        // Tracking variables
        double minVoltage = 3.3;
        double maxVoltage = 0.0;
        double lastAngle = 0.0;
        double lastTime = runtime.seconds();
        double velocity = 0.0;

        long incrementStartTime = 0;
        boolean isIncrementing = false;
        double incrementDirection = 0;

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();

            // Read analog data if available
            double voltage = 0;
            double angle = 0;

            if (rampAnalog != null) {
                voltage = rampAnalog.getVoltage();
                angle = voltage * VOLTAGE_TO_DEGREES;

                // Track min/max
                if (voltage < minVoltage) minVoltage = voltage;
                if (voltage > maxVoltage) maxVoltage = voltage;

                // Calculate velocity (degrees per second)
                double deltaTime = currentTime - lastTime;
                if (deltaTime > 0.1) {  // Update every 0.1 seconds
                    double deltaAngle = angle - lastAngle;

                    // Handle wraparound
                    if (deltaAngle > 180) deltaAngle -= 360;
                    if (deltaAngle < -180) deltaAngle += 360;

                    velocity = deltaAngle / deltaTime;
                    lastAngle = angle;
                    lastTime = currentTime;
                }
            }

            // Control logic
            double power = 0;

            // Incremental control (small pulses)
            if (gamepad1.dpad_up && !lastDpadUp) {
                // Start increment up
                isIncrementing = true;
                incrementDirection = INCREMENT_POWER;
                incrementStartTime = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                // Start increment down
                isIncrementing = true;
                incrementDirection = -INCREMENT_POWER;
                incrementStartTime = System.currentTimeMillis();
            }

            // Handle increment timing
            if (isIncrementing) {
                if (System.currentTimeMillis() - incrementStartTime < INCREMENT_TIME_MS) {
                    power = incrementDirection;
                } else {
                    isIncrementing = false;
                    power = 0;
                }
            }

            // Continuous control (hold button)
            if (gamepad1.dpad_left) {
                power = -0.5;  // Continuous down
                isIncrementing = false;
            } else if (gamepad1.dpad_right) {
                power = 0.5;   // Continuous up
                isIncrementing = false;
            }

            // Stop button
            if (gamepad1.a) {
                power = 0;
                isIncrementing = false;
            }

            // Apply power
            if (rampServo != null) {
                rampServo.setPower(power);
            }

            // Update button states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // Display telemetry
            telemetry.addLine("=== RAMP STATUS ===");

            if (rampAnalog != null) {
                telemetry.addData("Voltage", "%.4f V", voltage);
                telemetry.addData("Angle", "%.1f°", angle);
                telemetry.addData("Velocity", "%.1f°/s", velocity);
                telemetry.addLine();
                telemetry.addData("Min Voltage", "%.4f V", minVoltage);
                telemetry.addData("Max Voltage", "%.4f V", maxVoltage);
                telemetry.addData("Range", "%.4f V", maxVoltage - minVoltage);
                telemetry.addData("Est. Angle Range", "%.0f°",
                        (maxVoltage - minVoltage) * VOLTAGE_TO_DEGREES);
            } else {
                telemetry.addLine("⚠No analog feedback");
            }

            telemetry.addLine();
            telemetry.addData("Servo Power", "%.2f", power);

            if (isIncrementing) {
                telemetry.addLine("⚡ PULSING " + (incrementDirection > 0 ? "UP" : "DOWN"));
            } else if (power > 0) {
                telemetry.addLine("🔄 Moving UP");
            } else if (power < 0) {
                telemetry.addLine("🔄 Moving DOWN");
            } else {
                telemetry.addLine("⏸️ STOPPED");
            }

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPAD UP: Small increment ↑");
            telemetry.addLine("DPAD DOWN: Small increment ↓");
            telemetry.addLine("DPAD RIGHT: Continuous ↑");
            telemetry.addLine("DPAD LEFT: Continuous ↓");
            telemetry.addLine("A: STOP");

            telemetry.update();
        }

        // Stop servo when done
        if (rampServo != null) {
            rampServo.setPower(0);
        }
    }
}