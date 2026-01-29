package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.champion.controller.*;

/**
 * Component Test TeleOp - Test each robot subsystem independently
 *
 * GAMEPAD 1 CONTROLS:
 * ------------------
 * INTAKE:
 *   A - Toggle intake on/off
 *   B - Toggle intake direction
 *
 * TRANSFER:
 *   X - Toggle transfer on/off
 *   Y - Toggle transfer direction
 *
 * UPTAKE:
 *   Left Bumper - Toggle uptake on/off
 *   Right Bumper - Toggle uptake direction
 *
 * SHOOTER: ramp-> turret, shooter not plugged, uptake not known, turret->uptake
 *   Right Trigger - Start/stop shooter
 *   Dpad Up - Increase target RPM
 *   Dpad Down - Decrease target RPM
 *   Dpad Left - Toggle motor 1 direction
 *   Dpad Right - Toggle motor 2 direction
 *
 * RAMP:
 *   Left Stick Y - Manual control (when not using angle control)
 *   Start - Initialize ramp to 0 degrees
 *   Back - Stop ramp
 *
 * GAMEPAD 2 CONTROLS:
 * ------------------
 * RAMP ANGLE CONTROL:
 *   Dpad Up - Increment angle by 5 degrees
 *   Dpad Down - Decrement angle by 5 degrees
 *   Left Bumper - Set angle to 0 degrees
 *   Right Bumper - Set angle to 90 degrees
 *
 * TURRET:
 *   Right Stick X - Manual turret control
 *   Start - Reset turret angle to 0
 */
@TeleOp(name = "Component Test", group = "Test")
public class ComponentTestTeleop extends LinearOpMode {

    // Controllers
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;
    private TurretController turret;

    // Button state tracking (for toggles)
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private boolean lastLB1 = false, lastRB1 = false, lastRT = false;
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;
    private boolean lastStart1 = false, lastBack1 = false;

    // Gamepad 2 button states
    private boolean lastDpadUp2 = false, lastDpadDown2 = false;
    private boolean lastLB2 = false, lastRB2 = false, lastStart2 = false;

    @Override
    public void runOpMode() {

        // Initialize all controllers
        initializeControllers();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update all controllers
            updateControllers();

            // Handle gamepad 1 inputs
            handleGamepad1();

            // Handle gamepad 2 inputs
            handleGamepad2();

            // Display telemetry
            displayTelemetry();
        }

        // Stop everything on exit
        stopAll();
    }

    private void initializeControllers() {
        // Intake
        try {
            DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intake = new NewIntakeController(intakeMotor);
        } catch (Exception e) {
            telemetry.addLine("Intake not found");
            intake = null;
        }

        // Transfer
        try {
            DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            transfer = new NewTransferController(transferMotor);
        } catch (Exception e) {
            telemetry.addLine("Transfer not found");
            transfer = null;
        }

        // Uptake
        try {
            CRServo uptakeServo1 = hardwareMap.get(CRServo.class, "servo1");
            CRServo uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
            uptake = new UptakeController(uptakeServo1, uptakeServo2);
        } catch (Exception e) {
            telemetry.addLine("Uptake not found");
            uptake = null;
        }

        // Shooter
        try {
            DcMotor shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter");
            DcMotor shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
            shooter = new NewShooterController(shooterMotor1);
        } catch (Exception e) {
            telemetry.addLine("Shooter not found");
            shooter = null;
        }

        // Ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception e) {
            telemetry.addLine("Ramp not found");
            ramp = null;
        }

        // Turret
        try {
            turret = new TurretController(this);
            turret.initialize();
        } catch (Exception e) {
            telemetry.addLine("Turret not found");
            turret = null;
        }
    }

    private void updateControllers() {
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
        if (ramp != null) ramp.update();
        if (turret != null) turret.update();
    }

    private void handleGamepad1() {
        // INTAKE - A/B buttons
        if (gamepad1.a && !lastA) {
            if (intake != null) intake.toggle();
        }
        if (gamepad1.b && !lastB) {
            if (intake != null) intake.toggleDirection();
        }

        // TRANSFER - X/Y buttons
        if (gamepad1.x && !lastX) {
            if (transfer != null) transfer.toggle();
        }
        if (gamepad1.y && !lastY) {
            if (transfer != null) transfer.toggleDirection();
        }

        // UPTAKE - Bumpers
        if (gamepad1.left_bumper && !lastLB1) {
            if (uptake != null) uptake.toggle();
        }
        if (gamepad1.right_bumper && !lastRB1) {
            if (uptake != null) uptake.toggleDirection();
        }

        // SHOOTER - Right trigger and dpad
        if (gamepad1.right_trigger > 0.5 && !lastRT) {
            if (shooter != null) shooter.toggleShoot();
        }
        if (gamepad1.dpad_up && !lastDpadUp) {
            if (shooter != null) shooter.incrementTargetRPM();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            if (shooter != null) shooter.decrementTargetRPM();
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            if (shooter != null) shooter.toggleShootDirection();
        }

        // RAMP - Manual control with left stick Y
        if (ramp != null) {
            double rampPower = -gamepad1.left_stick_y * 0.3; // Scale down for safety
            if (Math.abs(rampPower) > 0.1) {
                ramp.setPower(rampPower);
            }
        }

        // RAMP - Start to initialize
        if (gamepad1.start && !lastStart1) {
            if (ramp != null) ramp.initialize();
        }

        // RAMP - Back to stop
        if (gamepad1.back && !lastBack1) {
            if (ramp != null) ramp.stop();
        }

        // Update button states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastLB1 = gamepad1.left_bumper;
        lastRB1 = gamepad1.right_bumper;
        lastRT = gamepad1.right_trigger > 0.5;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastStart1 = gamepad1.start;
        lastBack1 = gamepad1.back;
    }

    private void handleGamepad2() {
        // RAMP ANGLE CONTROL
        if (ramp != null) {
            if (gamepad2.dpad_up && !lastDpadUp2) {
                ramp.incrementAngle(5.0);
            }
            if (gamepad2.dpad_down && !lastDpadDown2) {
                ramp.decrementAngle(5.0);
            }
            if (gamepad2.left_bumper && !lastLB2) {
                ramp.setTargetAngle(0.0);
            }
            if (gamepad2.right_bumper && !lastRB2) {
                ramp.setTargetAngle(90.0);
            }
        }

        // TURRET - Right stick X
        if (turret != null) {
            double turretPower = gamepad2.right_stick_x * 0.5; // Scale for control
            turret.setPower(turretPower);
        }

        // TURRET - Start to reset angle
        if (gamepad2.start && !lastStart2) {
            if (turret != null) turret.resetAngle();
        }

        // Update gamepad 2 button states
        lastDpadUp2 = gamepad2.dpad_up;
        lastDpadDown2 = gamepad2.dpad_down;
        lastLB2 = gamepad2.left_bumper;
        lastRB2 = gamepad2.right_bumper;
        lastStart2 = gamepad2.start;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== COMPONENT STATUS ===");
        telemetry.addLine();

        // Intake status
        if (intake != null) {
            telemetry.addLine(String.format("INTAKE: %s | Power: %.2f | Rev: %s",
                    intake.isActive() ? "ON" : "OFF",
                    intake.getCurrentPower(),
                    intake.reversed ? "YES" : "NO"));
        }

        // Transfer status
        if (transfer != null) {
            telemetry.addLine(String.format("TRANSFER: %s | Power: %.2f | Rev: %s",
                    transfer.isActive() ? "ON" : "OFF",
                    transfer.getCurrentPower(),
                    transfer.reversed ? "YES" : "NO"));
        }

        // Uptake status
        if (uptake != null) {
            telemetry.addLine(String.format("UPTAKE: %s | Power: %.2f | Rev: %s",
                    uptake.isActive() ? "ON" : "OFF",
                    uptake.getCurrentPower(),
                    uptake.reversed ? "YES" : "NO"));
        }

        // Shooter status
        if (shooter != null) {
            telemetry.addLine(String.format("SHOOTER: %s | Target: %.0f RPM | Current: %.0f RPM",
                    shooter.isShootMode() ? "ACTIVE" : "OFF",
                    shooter.getTargetRPM(),
                    shooter.getRPM()));
            telemetry.addLine(String.format("  Error: %.0f RPM | At Target: %s",
                    shooter.getRPMError(),
                    shooter.isAtTargetRPM() ? "YES" : "NO"));
        }

        // Ramp status
        if (ramp != null) {
            telemetry.addLine(String.format("RAMP: Target: %.1f° | Current: %.1f° | Error: %.1f°",
                    ramp.getTargetAngle(),
                    ramp.getCurrentAngle(),
                    ramp.getAngleError()));
            telemetry.addLine(String.format("  Moving: %s | Power: %.2f | Init: %s",
                    ramp.isMoving() ? "YES" : "NO",
                    ramp.getPower(),
                    ramp.isInitialized() ? "YES" : "NO"));
        }

        // Turret status
        if (turret != null) {
            telemetry.addLine(String.format("TURRET: Angle: %.1f° | Normalized: %.1f°",
                    turret.getTurretAngle(),
                    turret.getTurretAngleNormalized()));
            telemetry.addLine(String.format("  Power: %.2f | Rotations: %d",
                    turret.getPower(),
                    turret.getServoRotationCount()));
        }

        telemetry.addLine();
        telemetry.addLine("See code for control mappings");
        telemetry.update();
    }

    private void stopAll() {
        if (intake != null) {
            intake.setState(false);
            intake.update();
        }
        if (transfer != null) {
            transfer.stop();
            transfer.update();
        }
        if (uptake != null) {
            uptake.setState(false);
            uptake.update();
        }
        if (shooter != null) {
            shooter.stopShooting();
        }
        if (ramp != null) {
            ramp.stop();
        }
        if (turret != null) {
            turret.stop();
        }
    }
}