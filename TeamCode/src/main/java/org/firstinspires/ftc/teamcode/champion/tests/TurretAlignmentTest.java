package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretAlignmentController;

@TeleOp(name = "Turret Alignment Test", group = "Tests")
@Config
public class TurretAlignmentTest extends LinearOpMode {

    private TurretController turretController;
    private TurretAlignmentController alignmentController;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize turret hardware
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret_servo");
        AnalogInput turretEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder");

        turretController = new TurretController(turretServo, turretEncoder, runtime);

        // Initialize alignment controller
        try {
            alignmentController = new TurretAlignmentController(this, turretController);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize alignment controller: " + e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addLine("Turret Alignment Test Ready");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        alignmentController.setTargetTag(20); // Default target tag

        while (opModeIsActive()) {
            // Update turret
            turretController.update();

            // Manual turret control with gamepad
            double manualInput = gamepad1.left_stick_x;
            turretController.calculatePower(manualInput);

            // Start alignment on A button press
            if (gamepad1.a) {
                alignmentController.startAlignment();
            }

            // Stop alignment on B button
            if (gamepad1.b) {
                alignmentController.stopAlignment();
            }

            // Display telemetry
            telemetry.addLine("=== TURRET ALIGNMENT TEST ===");
            telemetry.addLine();

            telemetry.addLine(">>> TURret STATUS <<<");
            telemetry.addData("Current Position", "%.2f°", turretController.getCurrentPosition());
            telemetry.addData("Velocity", "%.2f°/s", turretController.getVelocity());
            telemetry.addData("Power", "%.3f", turretController.calculatePower(manualInput));
            telemetry.addLine();

            telemetry.addLine(">>> ALIGNMENT STATUS <<<");
            telemetry.addData("State", alignmentController.getState());
            telemetry.addData("Has Target", alignmentController.hasTarget());
            telemetry.addData("Target Error", "%.2f°", alignmentController.getTargetError());
            telemetry.addData("Zone", alignmentController.getCurrentZone());
            telemetry.addData("Total Time", "%.2f s", alignmentController.getTotalAlignmentTime());
            telemetry.addLine();

            telemetry.addLine(">>> CONTROLS <<<");
            telemetry.addLine("Left Stick X: Manual turret control");
            telemetry.addLine("A: Start alignment");
            telemetry.addLine("B: Stop alignment");
            telemetry.addLine();

            alignmentController.displayAlignmentWithInitialAngle();

            telemetry.update();

            sleep(20);
        }
    }
}