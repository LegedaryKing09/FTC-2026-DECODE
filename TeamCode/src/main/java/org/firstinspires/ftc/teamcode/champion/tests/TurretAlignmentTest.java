package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.TurretAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;

@TeleOp(name="Turret Alignment Test", group="Test")
public class TurretAlignmentTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize turret
        CRServo turretServo = hardwareMap.get(CRServo.class, "turret");
        AnalogInput turretEncoder = hardwareMap.get(AnalogInput.class, "turret_analog");
        TurretController turret = new TurretController(turretServo, turretEncoder, runtime);

        // Initialize alignment controller
        TurretAlignmentController turretAlignment;
        try {
            turretAlignment = new TurretAlignmentController(this, turret);
            turretAlignment.setTargetTag(20);
            telemetry.addLine("✓ Turret alignment ready");
        } catch (Exception e) {
            telemetry.addLine("✗ Failed to initialize: " + e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addLine("Press A to align to tag");
        telemetry.addLine("Press B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // A button - start alignment
            if (gamepad1.a) {
                turretAlignment.startAlignment();
            }

            // B button - stop
            if (gamepad1.b) {
                turretAlignment.stopAlignment();
            }

            // Update controllers
            turret.update();
            turretAlignment.update();

            // Display status
            telemetry.addLine("A = Align | B = Stop");
            telemetry.addLine();
            turretAlignment.displayTelemetry();
        }
    }
}