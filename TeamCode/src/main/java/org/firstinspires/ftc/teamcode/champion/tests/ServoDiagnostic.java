package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Standalone Servo Diagnostic Test
 *
 * Tests each servo independently to diagnose:
 * - Directionality issues (servos fighting each other)
 * - SuperServo mode problems
 * - Power/connection issues
 * - PWM signal issues
 *
 * CONTROLS:
 * ---------
 * A - Run servo1 FORWARD only
 * B - Run servo1 BACKWARD only
 * X - Run servo2 FORWARD only
 * Y - Run servo2 BACKWARD only
 *
 * Left Bumper  - Run BOTH forward (same direction)
 * Right Bumper - Run BOTH forward but servo2 REVERSED
 *
 * Dpad Up   - Increase power (0.1 increments)
 * Dpad Down - Decrease power (0.1 increments)
 *
 * Left Trigger  - Run servo1 at variable speed (trigger amount)
 * Right Trigger - Run servo2 at variable speed (trigger amount)
 *
 * START - Toggle between CRServo mode and standard Servo mode test
 * BACK  - Stop everything
 */
@TeleOp(name = "Servo Diagnostic Test", group = "Test")
public class ServoDiagnostic extends LinearOpMode {

    private CRServo crServo1 = null;
    private CRServo crServo2 = null;

    private double testPower = 0.5;
    private boolean servo1Found = false;
    private boolean servo2Found = false;

    // Track what's running for telemetry
    private String servo1Status = "STOPPED";
    private String servo2Status = "STOPPED";

    @Override
    public void runOpMode() {

        // Try to initialize servo1
        try {
            crServo1 = hardwareMap.get(CRServo.class, "servo1");
            servo1Found = true;
            telemetry.addLine("✓ servo1 found");
        } catch (Exception e) {
            telemetry.addLine("✗ servo1 NOT FOUND: " + e.getMessage());
        }

        // Try to initialize servo2
        try {
            crServo2 = hardwareMap.get(CRServo.class, "servo2");
            servo2Found = true;
            telemetry.addLine("✓ servo2 found");
        } catch (Exception e) {
            telemetry.addLine("✗ servo2 NOT FOUND: " + e.getMessage());
        }

        telemetry.addLine();
        telemetry.addLine("=== SERVO DIAGNOSTIC TEST ===");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Default: stop both servos
            double power1 = 0;
            double power2 = 0;
            servo1Status = "STOPPED";
            servo2Status = "STOPPED";

            // ===== INDIVIDUAL SERVO TESTS =====

            // A - Servo1 forward only
            if (gamepad1.a) {
                power1 = testPower;
                servo1Status = "FORWARD (" + String.format("%.1f", testPower) + ")";
            }

            // B - Servo1 backward only
            if (gamepad1.b) {
                power1 = -testPower;
                servo1Status = "BACKWARD (" + String.format("%.1f", -testPower) + ")";
            }

            // X - Servo2 forward only
            if (gamepad1.x) {
                power2 = testPower;
                servo2Status = "FORWARD (" + String.format("%.1f", testPower) + ")";
            }

            // Y - Servo2 backward only
            if (gamepad1.y) {
                power2 = -testPower;
                servo2Status = "BACKWARD (" + String.format("%.1f", -testPower) + ")";
            }

            // ===== COMBINED TESTS =====

            // Left Bumper - Both same direction
            if (gamepad1.left_bumper) {
                power1 = testPower;
                power2 = testPower;
                servo1Status = "FORWARD (BOTH SAME)";
                servo2Status = "FORWARD (BOTH SAME)";
            }

            // Right Bumper - Both forward but servo2 inverted
            if (gamepad1.right_bumper) {
                power1 = testPower;
                power2 = -testPower;  // Inverted!
                servo1Status = "FORWARD (COMBINED)";
                servo2Status = "REVERSED (COMBINED)";
            }

            // ===== VARIABLE SPEED WITH TRIGGERS =====

            if (gamepad1.left_trigger > 0.1) {
                power1 = gamepad1.left_trigger;
                servo1Status = "TRIGGER (" + String.format("%.2f", power1) + ")";
            }

            if (gamepad1.right_trigger > 0.1) {
                power2 = gamepad1.right_trigger;
                servo2Status = "TRIGGER (" + String.format("%.2f", power2) + ")";
            }

            // ===== POWER ADJUSTMENT =====

            if (gamepad1.dpad_up) {
                testPower = Math.min(1.0, testPower + 0.01);
                sleep(50); // Debounce
            }
            if (gamepad1.dpad_down) {
                testPower = Math.max(0.1, testPower - 0.01);
                sleep(50); // Debounce
            }

            // ===== EMERGENCY STOP =====

            if (gamepad1.back) {
                power1 = 0;
                power2 = 0;
                servo1Status = "E-STOP";
                servo2Status = "E-STOP";
            }

            // ===== APPLY POWER =====

            if (servo1Found && crServo1 != null) {
                crServo1.setPower(power1);
            }
            if (servo2Found && crServo2 != null) {
                crServo2.setPower(power2);
            }

            // ===== TELEMETRY =====

            telemetry.addLine("=== SERVO DIAGNOSTIC TEST ===");
            telemetry.addLine();

            telemetry.addLine("SERVO 1: " + (servo1Found ? "CONNECTED" : "NOT FOUND"));
            telemetry.addLine("  Status: " + servo1Status);
            telemetry.addLine("  Power:  " + String.format("%.2f", power1));
            telemetry.addLine();

            telemetry.addLine("SERVO 2: " + (servo2Found ? "CONNECTED" : "NOT FOUND"));
            telemetry.addLine("  Status: " + servo2Status);
            telemetry.addLine("  Power:  " + String.format("%.2f", power2));
            telemetry.addLine();

            telemetry.addLine("Test Power: " + String.format("%.1f", testPower));
            telemetry.addLine();

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("A/B: Servo1 fwd/back");
            telemetry.addLine("X/Y: Servo2 fwd/back");
            telemetry.addLine("LB: Both same direction");
            telemetry.addLine("RB: Both opposite (servo2 reversed)");
            telemetry.addLine("Triggers: Variable speed");
            telemetry.addLine("Dpad: Adjust test power");
            telemetry.addLine("BACK: Emergency stop");

            telemetry.update();
        }

        // Stop on exit
        if (crServo1 != null) crServo1.setPower(0);
        if (crServo2 != null) crServo2.setPower(0);
    }
}