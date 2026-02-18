package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * RAMP ANGLE TESTER
 *
 * Tests ramp angle tracking and increment/decrement functions.
 *
 * CONTROLS:
 * - A = Reset (current position = 0°)
 * - LB = Increment angle by INCREMENT_DEGREES
 * - LT = Decrement angle by INCREMENT_DEGREES
 * - B = Apply positive power manually
 * - X = Apply negative power manually
 * - Y = Set target to -100° (test preset)
 *
 * Shows all intermediate values for debugging.
 */
@Config
@TeleOp(name = "!! Ramp Angle Tester !!", group = "Test")
public class RampAngle extends LinearOpMode {

    public static String RAMP_SERVO_NAME = "ramp";
    public static String RAMP_ANALOG_NAME = "ramp_analog";

    public static double VOLTAGE_TO_DEGREES = 360.0 / 3.3;
    public static double INCREMENT_DEGREES = 5.0;  // Positive value!

    // PID
    public static double Kp = 0.03;
    public static double MAX_POWER = 0.1;
    public static double MIN_POWER = 0.05;
    public static double ANGLE_TOLERANCE = 4.0;
    public static boolean INVERT_OUTPUT = true;

    private CRServo servo;
    private AnalogInput analog;

    // Delta-based tracking
    private double lastRawAngle = 0;
    private double accumulatedAngle = 0;
    private int rotationCount = 0;

    // PID state
    private double targetAngle = 0;
    private boolean pidActive = false;

    // Button states
    private boolean lastA = false;
    private boolean lastLB = false;
    private boolean lastLT = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            servo = hardwareMap.get(CRServo.class, RAMP_SERVO_NAME);
            analog = hardwareMap.get(AnalogInput.class, RAMP_ANALOG_NAME);
        } catch (Exception e) {
            telemetry.addLine("ERROR: " + e.getMessage());
            telemetry.update();
            waitForStart();
            return;
        }

        servo.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("=== RAMP ANGLE TESTER ===");
        telemetry.addLine("LB = increment, LT = decrement");
        telemetry.update();

        waitForStart();

        // Initialize
        lastRawAngle = getRawAngle();
        accumulatedAngle = 0;
        targetAngle = 0;

        while (opModeIsActive()) {
            // Update angle tracking (delta-based)
            double currentRawAngle = getRawAngle();
            double rawDelta = currentRawAngle - lastRawAngle;

            // Handle wraparound
            if (rawDelta < -180) {
                rawDelta += 360;
                rotationCount++;
            } else if (rawDelta > 180) {
                rawDelta -= 360;
                rotationCount--;
            }

            accumulatedAngle += rawDelta;
            lastRawAngle = currentRawAngle;

            // === A: RESET ===
            if (gamepad1.a && !lastA) {
                lastRawAngle = currentRawAngle;
                accumulatedAngle = 0;
                targetAngle = 0;
                rotationCount = 0;
                pidActive = false;
            }
            lastA = gamepad1.a;

            // === LB: INCREMENT ===
            if (gamepad1.left_bumper && !lastLB) {
                targetAngle = accumulatedAngle + INCREMENT_DEGREES;
                pidActive = true;
                telemetry.addLine(">>> LB PRESSED: target = " + targetAngle);
            }
            lastLB = gamepad1.left_bumper;

            // === LT: DECREMENT ===
            boolean ltPressed = gamepad1.left_trigger > 0.3;
            if (ltPressed && !lastLT) {
                targetAngle = accumulatedAngle - INCREMENT_DEGREES;
                pidActive = true;
                telemetry.addLine(">>> LT PRESSED: target = " + targetAngle);
            }
            lastLT = ltPressed;

            // === Y: PRESET TEST ===
            if (gamepad1.y && !lastY) {
                targetAngle = -100.0;
                pidActive = true;
            }
            lastY = gamepad1.y;

            // === B/X: MANUAL POWER ===
            double manualPower = 0;
            if (gamepad1.b) {
                manualPower = 0.15;
                pidActive = false;
            } else if (gamepad1.x) {
                manualPower = -0.15;
                pidActive = false;
            }

            // === PID CONTROL ===
            double power = manualPower;
            double error = targetAngle - accumulatedAngle;

            if (pidActive && manualPower == 0) {
                // Simple P control
                power = Kp * error;

                // Min power
                if (Math.abs(power) > 0.01 && Math.abs(power) < MIN_POWER) {
                    power = Math.signum(power) * MIN_POWER;
                }

                // Clamp
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                // Check if done
                if (Math.abs(error) < ANGLE_TOLERANCE) {
                    pidActive = false;
                    power = 0;
                }
            }

            // Apply inversion
            double finalPower = INVERT_OUTPUT ? -power : power;
            servo.setPower(finalPower);

            // === TELEMETRY ===
            telemetry.addLine("=== RAMP ANGLE TESTER ===");
            telemetry.addLine();

            telemetry.addLine("--- RAW SENSOR ---");
            telemetry.addData("Voltage", "%.3f V", analog.getVoltage());
            telemetry.addData("Raw Angle", "%.1f°", currentRawAngle);
            telemetry.addLine();

            telemetry.addLine("--- TRACKING ---");
            telemetry.addData("Accumulated Angle", "%.1f°", accumulatedAngle);
            telemetry.addData("Rotation Count", rotationCount);
            telemetry.addLine();

            telemetry.addLine("--- PID ---");
            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("PID Active", pidActive);
            telemetry.addData("Power (before invert)", "%.3f", power);
            telemetry.addData("Power (final)", "%.3f", finalPower);
            telemetry.addData("INVERT_OUTPUT", INVERT_OUTPUT);
            telemetry.addLine();

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("INCREMENT_DEGREES", "%.1f", INCREMENT_DEGREES);
            telemetry.addLine("A=Reset | LB=+Inc | LT=-Inc");
            telemetry.addLine("B=+Power | X=-Power | Y=Preset(-100)");

            telemetry.update();
            sleep(50);
        }
    }

    private double getRawAngle() {
        return analog.getVoltage() * VOLTAGE_TO_DEGREES;
    }
}