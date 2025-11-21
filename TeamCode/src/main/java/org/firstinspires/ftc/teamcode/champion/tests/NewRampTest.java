package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.newRampController;

@TeleOp(name = "New Ramp Test")
public class NewRampTest extends LinearOpMode {

    private newRampController rampController;

    // Button state tracking (following AutoTeleop pattern)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

    // Angle increment per button press
    private static final double ANGLE_INCREMENT = 5.0;

    @Override
    public void runOpMode() {
        // Initialize the ramp controller
        rampController = new newRampController(this);

        telemetry.addLine("=== New Ramp Controller Test ===");
        telemetry.addLine("Waiting for start...");
        telemetry.addLine();
        telemetry.addLine("A: Increase angle by 5°");
        telemetry.addLine("B: Decrease angle by 5°");
        telemetry.addLine("X: Emergency Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: Update the ramp controller every loop
            rampController.update();

            // A button: Increase angle by 5 degrees
            if (gamepad1.a && !lastA) {
                rampController.incrementAngle(ANGLE_INCREMENT);
            }
            lastA = gamepad1.a;

            // B button: Decrease angle by 5 degrees
            if (gamepad1.b && !lastB) {
                rampController.decrementAngle(ANGLE_INCREMENT);
            }
            lastB = gamepad1.b;

            // X button: Emergency stop
            if (gamepad1.x && !lastX) {
                rampController.stop();
            }
            lastX = gamepad1.x;

            // Display telemetry
            telemetry.addLine("=== NEW RAMP STATUS ===");
            telemetry.addData("Current Angle", "%.1f°", rampController.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", rampController.getTargetAngle());
            telemetry.addData("Error", "%.2f°", rampController.getAngleError());
            telemetry.addData("Velocity", "%.1f°/s", rampController.getVelocity());
            telemetry.addLine();

            telemetry.addData("Servo Power", "%.3f", rampController.getPower());
            telemetry.addData("Raw Voltage", "%.3f V", rampController.getVoltage());
            telemetry.addLine();

            telemetry.addData("Status", rampController.isMoving() ? "MOVING" : "IDLE");
            telemetry.addData("At Target", rampController.isAtTargetAngle() ? "✅ YES" : "NO");

            if (!rampController.isMoving()) {
                telemetry.addData("Final Error", "%.4f°", rampController.getFinalError());
            }
            telemetry.addLine();

            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A: Increase 5°");
            telemetry.addLine("B: Decrease 5°");
            telemetry.addLine("X: Emergency Stop");

            telemetry.update();
        }

        // Safety: Stop ramp when OpMode ends
        rampController.stop();
    }
}