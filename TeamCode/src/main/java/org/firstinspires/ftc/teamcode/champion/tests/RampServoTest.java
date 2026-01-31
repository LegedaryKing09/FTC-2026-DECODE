package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Simple Raw Ramp Servo Test
 *
 * NO angle conversion - just raw servo position control.
 * Use this to verify servo works correctly.
 *
 * YOUR CALIBRATION:
 * Servo 0.0 → Raw angle 90°, Voltage 0.831V (EXTENDED)
 * Servo 1.0 → Raw angle 334.8°, Voltage 3.069V (RETRACTED)
 *
 * CONTROLS:
 * ---------
 * Left Stick Y  - Manual servo control (fast)
 * Right Stick Y - Manual servo control (fine/slow)
 *
 * A - Go to 0.0 (Extended)
 * B - Go to 0.5 (Middle)
 * Y - Go to 1.0 (Retracted)
 *
 * Dpad Up    - Servo +0.05
 * Dpad Down  - Servo -0.05
 * Dpad Right - Servo +0.01
 * Dpad Left  - Servo -0.01
 *
 * Right Bumper - Servo +0.1 (big step)
 * Left Bumper  - Servo -0.1 (big step)
 */
@Config
@TeleOp(name = "Ramp Servo RAW Test", group = "Test")
public class RampServoTest extends LinearOpMode {

    public static String SERVO_NAME = "ramp";
    public static String ANALOG_NAME = "ramp_analog";
    public static double VOLTAGE_MAX = 3.3;

    private Servo rampServo;
    private AnalogInput rampAnalog;

    // Direct servo position - no conversions!
    private double servoPosition = 0.5;

    // Button tracking
    private boolean lastA = false, lastB = false, lastY = false;
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;
    private boolean lastLB = false, lastRB = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        try {
            rampServo = hardwareMap.get(Servo.class, SERVO_NAME);
            telemetry.addLine("✓ Servo found");
        } catch (Exception e) {
            telemetry.addLine("✗ Servo NOT FOUND");
            rampServo = null;
        }

        try {
            rampAnalog = hardwareMap.get(AnalogInput.class, ANALOG_NAME);
            telemetry.addLine("✓ Analog found");
        } catch (Exception e) {
            telemetry.addLine("✗ Analog NOT FOUND");
            rampAnalog = null;
        }

        // Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine();
        telemetry.addLine("=== RAW SERVO TEST ===");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        // Start at middle
        servoPosition = 0.5;
        if (rampServo != null) {
            rampServo.setPosition(servoPosition);
        }

        while (opModeIsActive()) {

            // === JOYSTICK CONTROL ===

            // Left stick - fast manual
            double leftY = -gamepad1.left_stick_y;
            if (Math.abs(leftY) > 0.1) {
                servoPosition += leftY * 0.03;
            }

            // Right stick - fine manual
            double rightY = -gamepad1.right_stick_y;
            if (Math.abs(rightY) > 0.1) {
                servoPosition += rightY * 0.008;
            }

            // === BUTTON PRESETS ===

            if (gamepad1.a && !lastA) {
                servoPosition = 0.0;  // Extended
            }
            if (gamepad1.b && !lastB) {
                servoPosition = 0.5;  // Middle
            }
            if (gamepad1.y && !lastY) {
                servoPosition = 1.0;  // Retracted
            }

            // === DPAD INCREMENTS ===

            if (gamepad1.dpad_up && !lastDpadUp) {
                servoPosition += 0.05;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                servoPosition -= 0.05;
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                servoPosition += 0.01;
            }
            if (gamepad1.dpad_left && !lastDpadLeft) {
                servoPosition -= 0.01;
            }

            // === BUMPER BIG STEPS ===

            if (gamepad1.right_bumper && !lastRB) {
                servoPosition += 0.1;
            }
            if (gamepad1.left_bumper && !lastLB) {
                servoPosition -= 0.1;
            }

            // === CLAMP AND APPLY ===

            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            if (rampServo != null) {
                rampServo.setPosition(servoPosition);
            }

            // === UPDATE BUTTON STATES ===

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // === TELEMETRY ===

            telemetry.addLine("=== RAW SERVO TEST ===");
            telemetry.addLine();

            telemetry.addLine("--- SERVO COMMAND ---");
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addLine();

            telemetry.addLine("--- ANALOG FEEDBACK ---");
            if (rampAnalog != null) {
                double voltage = rampAnalog.getVoltage();
                double rawAngle = (voltage / VOLTAGE_MAX) * 360.0;
                telemetry.addData("Voltage", "%.3f V", voltage);
                telemetry.addData("Raw Angle", "%.1f°", rawAngle);
            } else {
                telemetry.addLine("Analog not connected");
            }
            telemetry.addLine();

            telemetry.addLine("--- CALIBRATION REF ---");
            telemetry.addLine("0.0 → 90°, 0.83V (Extended)");
            telemetry.addLine("1.0 → 335°, 3.07V (Retracted)");
            telemetry.addLine();

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("Left Stick: Fast manual");
            telemetry.addLine("Right Stick: Fine manual");
            telemetry.addLine("A=0.0  B=0.5  Y=1.0");
            telemetry.addLine("Dpad U/D: ±0.05");
            telemetry.addLine("Dpad L/R: ±0.01");
            telemetry.addLine("Bumpers: ±0.1");

            telemetry.update();
        }

        // Stop
        if (rampServo != null) {
            rampServo.setPosition(servoPosition);
        }
    }
}