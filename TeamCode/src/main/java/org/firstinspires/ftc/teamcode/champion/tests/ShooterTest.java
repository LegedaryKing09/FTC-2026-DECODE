package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    private NewShooterController shooterController;
    private ElapsedTime runtime = new ElapsedTime();

    // Custom RPM control variables
    private double customRPM = 1000;
    private final double RPM_INCREMENT = 100;
    private final double MIN_RPM = 0;
    private final double MAX_RPM = 5000;

    // Button state tracking to prevent multiple triggers
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing Shooter...");
        telemetry.update();

        // Initialize the shooter controller
        try {
            shooterController = new NewShooterController(this);
            telemetry.addData("Status", "Shooter Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter: " + e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Ready to start");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A - Full Speed (3000 RPM)");
        telemetry.addLine("B - Half Speed (1500 RPM)");
        telemetry.addLine("X - Quarter Speed (750 RPM)");
        telemetry.addLine("Y - Stop Shooter");
        telemetry.addLine("DPAD UP/DOWN - Adjust Custom RPM");
        telemetry.addLine("LEFT/RIGHT BUMPER - Set Custom RPM");
        telemetry.addLine("LEFT TRIGGER - Fine RPM Control");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =================
            // GAMEPAD CONTROLS
            // =================

            // Preset RPM controls
            if (gamepad1.a) {
                shooterController.shooterFull();
                telemetry.addLine(">>> FULL SPEED ACTIVATED <<<");
            }
            else if (gamepad1.b) {
                shooterController.shooterHalf();
                telemetry.addLine(">>> HALF SPEED ACTIVATED <<<");
            }
            else if (gamepad1.x) {
                shooterController.shooterQuarter();
                telemetry.addLine(">>> QUARTER SPEED ACTIVATED <<<");
            }
            else if (gamepad1.y) {
                shooterController.shooterStop();
                telemetry.addLine(">>> SHOOTER STOPPED <<<");
            }

            // Custom RPM adjustment with DPAD
            if (gamepad1.dpad_up && !lastDpadUp) {
                customRPM = Math.min(customRPM + RPM_INCREMENT, MAX_RPM);
                telemetry.addLine("Custom RPM increased to: " + customRPM);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                customRPM = Math.max(customRPM - RPM_INCREMENT, MIN_RPM);
                telemetry.addLine("Custom RPM decreased to: " + customRPM);
            }

            // Set custom RPM with bumpers
            if (gamepad1.left_bumper && !lastLeftBumper) {
                shooterController.setShooterRPM(customRPM);
                telemetry.addLine(">>> CUSTOM RPM SET: " + customRPM + " <<<");
            }

            // Emergency stop with right bumper
            if (gamepad1.right_bumper && !lastRightBumper) {
                shooterController.shooterStop();
                telemetry.addLine(">>> EMERGENCY STOP <<<");
            }

            // Fine RPM control with left trigger (0-100% of max RPM)
            if (gamepad1.left_trigger > 0.1) {
                double triggerRPM = gamepad1.left_trigger * NewShooterController.SHOOTER_FULL_RPM;
                shooterController.setShooterRPM(triggerRPM);
                telemetry.addData("Trigger Control", "%.0f RPM (%.1f%%)", triggerRPM, gamepad1.left_trigger * 100);
            }

            // Update button states for edge detection
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            // =================
            // TELEMETRY OUTPUT
            // =================

            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.addLine("====================");

            // Add shooter telemetry
            shooterController.addTelemetry(telemetry);

            telemetry.addLine("====================");
            telemetry.addData("Custom RPM Setting", "%.0f", customRPM);
            telemetry.addData("Trigger Input", "%.3f (%.0f RPM)",
                    gamepad1.left_trigger,
                    gamepad1.left_trigger * NewShooterController.SHOOTER_FULL_RPM);

            // Performance indicators
            if (shooterController.isAtTargetRPM()) {
                telemetry.addLine("✓ SHOOTER AT TARGET RPM");
            } else if (shooterController.isDoingShooter()) {
                telemetry.addLine("⚡ SHOOTER RAMPING UP...");
            }

            // Current control scheme reminder
            telemetry.addLine("====================");
            telemetry.addLine("ACTIVE CONTROLS:");
            telemetry.addLine("A=Full | B=Half | X=Quarter | Y=Stop");
            telemetry.addLine("DPAD ↕=Adjust | LB=Set Custom | RB=E-Stop");
            telemetry.addLine("LT=Fine Control");

            telemetry.update();

            // Small delay to prevent overwhelming the system
            sleep(20);
        }

        // Cleanup - stop shooter when OpMode ends
        shooterController.shooterStop();
        telemetry.addLine("OpMode Ended - Shooter Stopped");
        telemetry.update();
    }
}