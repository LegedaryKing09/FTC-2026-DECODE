package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SimpleTurretAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;

@Config
@TeleOp(name = "Turret Alignment Test", group = "Test")
public class TurretAlignmentTest extends LinearOpMode {

    private TurretController turret;
    private SimpleTurretAlignmentController alignment;

    private final ElapsedTime runtime = new ElapsedTime();

    // Button state tracking
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // Setup telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("🔧 Initializing Turret Alignment Test...");
        telemetry.update();

        // Initialize turret
        try {
            turret = new TurretController(this);
            telemetry.addData("✓ Turret", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Turret", "FAILED: " + e.getMessage());
        }

        // Initialize alignment controller
        try {
            alignment = new SimpleTurretAlignmentController(this, turret);
            telemetry.addData("✓ Alignment", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Alignment", "FAILED: " + e.getMessage());
        }

        telemetry.addLine();
        telemetry.addLine("📋 CONTROLS:");
        telemetry.addLine("  Left Stick X = Manual turret control");
        telemetry.addLine("  Left Bumper = Start alignment");
        telemetry.addLine("  Right Bumper = Stop alignment");
        telemetry.addLine();
        telemetry.addLine("✅ Ready to start!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            handleControls();
            updateSystems();
            displayTelemetry();
        }
    }

    private void handleControls() {
        // Manual turret control - only when alignment is not running
        if (turret != null && !alignment.isRunning()) {
            double turretInput = gamepad1.left_stick_x;
            if (Math.abs(turretInput) > 0.1) {
                turret.setPower(turretInput);
            } else {
                turret.setPower(0);
            }
        }

        // Left bumper - start alignment
        boolean currentLB = gamepad1.left_bumper;
        if (currentLB && !lastLeftBumper) {
            if (alignment != null) {
                alignment.start();
            }
        }
        lastLeftBumper = currentLB;

        // Right bumper - stop alignment
        boolean currentRB = gamepad1.right_bumper;
        if (currentRB && !lastRightBumper) {
            if (alignment != null) {
                alignment.stop();
            }
        }
        lastRightBumper = currentRB;
    }

    private void updateSystems() {
        if (alignment != null) {
            alignment.update();
        }
    }

    private void displayTelemetry() {
        telemetry.addData("⏱ Runtime", "%.1f sec", runtime.seconds());
        telemetry.addLine();

        // Turret info
        if (turret != null) {
            telemetry.addLine("═══ TURRET ═══");
            telemetry.addData("Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("Power", "%.2f", turret.getPower());
        }

        telemetry.addLine();

        // Alignment info
        if (alignment != null) {
            telemetry.addLine("═══ ALIGNMENT ═══");
            telemetry.addData("Running", alignment.isRunning() ? "YES" : "NO");
            telemetry.addData("Target Tag ID", SimpleTurretAlignmentController.TARGET_TAG_ID);
            telemetry.addData("Tolerance", "%.1f°", SimpleTurretAlignmentController.TOLERANCE_DEGREES);
            telemetry.addData("Max Power", "%.2f", SimpleTurretAlignmentController.MAX_TURN_POWER);
            telemetry.addData("Min Power", "%.2f", SimpleTurretAlignmentController.MIN_TURN_POWER);
            telemetry.addData("Slowdown At", "%.1f°", SimpleTurretAlignmentController.SLOWDOWN_THRESHOLD);
        }

        telemetry.addLine();
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addData("Manual Control", alignment != null && !alignment.isRunning() ? "ENABLED" : "DISABLED");
        telemetry.addLine("LB = Start Align | RB = Stop");

        telemetry.update();
    }
}
