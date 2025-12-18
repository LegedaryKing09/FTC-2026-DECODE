package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;

@Config
@TeleOp(name = "New Shooter Tester", group = "Test")
public class NewShooterTester extends LinearOpMode {

    // Tunable via FTC Dashboard
    public static double RPM_INCREMENT = 50.0;
    public static double INITIAL_TARGET_RPM = 1500.0;

    //  New parameters to enable/disable individual motors
    public static boolean ENABLE_MOTOR_1 = true;  // Motor with encoder
    public static boolean ENABLE_MOTOR_2 = true;  // Motor without encoder

    private NewShooterController shooter;
    private final ElapsedTime runtime = new ElapsedTime();

    // Button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    // New button debouncing for motor toggle
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        // Setup telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   NEW SHOOTER TESTER");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize shooter
        DcMotor shooterMotorFirst = null;
        DcMotor shooterMotorSecond = null;
        try {
            shooterMotorFirst = hardwareMap.get(DcMotor.class, "shooter1");
            telemetry.addData("✓ Shooter Motor 1", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter Motor 1", "NOT FOUND: " + e.getMessage());
        }

        try {
            shooterMotorSecond = hardwareMap.get(DcMotor.class, "shooter2");
            telemetry.addData("✓ Shooter Motor 2", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter Motor 2", "NOT FOUND: " + e.getMessage());
        }

        shooter = new NewShooterController(shooterMotorFirst, shooterMotorSecond);
        shooter.setTargetRPM(INITIAL_TARGET_RPM);

        // Display controls
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   CONTROLS");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("DPAD UP:    +50 RPM");
        telemetry.addLine("DPAD DOWN:  -50 RPM");
        telemetry.addLine("DPAD LEFT:  Toggle Motor 1");  // CHANGE: New control
        telemetry.addLine("DPAD RIGHT: Toggle Motor 2");  // CHANGE: New control
        telemetry.addLine("Y:          START shooter");
        telemetry.addLine("X:          STOP shooter");
        telemetry.addLine("A:          Reset to " + INITIAL_TARGET_RPM + " RPM");
        telemetry.addLine("B:          Toggle direction");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ========== RPM ADJUSTMENT ==========

            // DPAD UP - Increase target RPM
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                double newRPM = shooter.getTargetRPM() + RPM_INCREMENT;
                shooter.setTargetRPM(newRPM);
            }
            lastDpadUp = currentDpadUp;

            // DPAD DOWN - Decrease target RPM
            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                double newRPM = shooter.getTargetRPM() - RPM_INCREMENT;
                shooter.setTargetRPM(newRPM);
            }
            lastDpadDown = currentDpadDown;

            // DPAD LEFT - Toggle Motor 1
            boolean currentDpadLeft = gamepad1.dpad_left;
            if (currentDpadLeft && !lastDpadLeft) {
                ENABLE_MOTOR_1 = !ENABLE_MOTOR_1;
            }
            lastDpadLeft = currentDpadLeft;

            // DPAD RIGHT - Toggle Motor 2
            boolean currentDpadRight = gamepad1.dpad_right;
            if (currentDpadRight && !lastDpadRight) {
                ENABLE_MOTOR_2 = !ENABLE_MOTOR_2;
            }
            lastDpadRight = currentDpadRight;

            // ========== SHOOTER CONTROL ==========

            // Y Button - Start shooter
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                shooter.startShooting();
            }
            lastY = currentY;

            // X Button - Stop shooter
            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                shooter.stopShooting();
            }
            lastX = currentX;

            // A Button - Reset to initial RPM
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                shooter.setTargetRPM(INITIAL_TARGET_RPM);
            }
            lastA = currentA;

            // B Button - Toggle direction
            boolean currentB = gamepad1.b;
            if (currentB && !lastB) {
                shooter.toggleShootDirection();
            }
            lastB = currentB;

            // ========== UPDATE CONTROLLER ==========
            shooter.update();

            // ========== TELEMETRY ==========
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("   SHOOTER STATUS");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine();

            // Mode indicator
            String modeStatus;
            if (shooter.isShootMode()) {
                if (shooter.isAtTargetRPM()) {
                    modeStatus = "🟢 RUNNING - AT TARGET";
                } else {
                    modeStatus = "🟡 RUNNING - RAMPING";
                }
            } else {
                modeStatus = "⚫ STOPPED";
            }
            telemetry.addData("Mode", modeStatus);
            telemetry.addLine();

            // Motor enable status
            telemetry.addData("Motor 1 (encoder)", ENABLE_MOTOR_1 ? "✓ ENABLED" : "✗ DISABLED");
            telemetry.addData("Motor 2 (no encoder)", ENABLE_MOTOR_2 ? "✓ ENABLED" : "✗ DISABLED");
            telemetry.addLine();

            // RPM data
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
            telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
            telemetry.addData("RPM Error", "%.0f", shooter.getRPMError());
            telemetry.addLine();

            // Visual RPM bar (0-6000 scale)
            double rpmPercent = (shooter.getRPM() / 6000.0) * 100.0;
            telemetry.addData("RPM Bar", "%.1f%%", rpmPercent);
            telemetry.addLine(createProgressBar(shooter.getRPM(), shooter.getTargetRPM(), 6000));
            telemetry.addLine();

            // Motor data for both motors
            telemetry.addData("Motor 1 Power", "%.3f", shooter.getCurrentPower());
            telemetry.addData("Motor 2 Power", "%.3f", shooter.getSecondMotorPower());
            telemetry.addData("Encoder Pos", "%d", shooter.getEncoderPosition());
            telemetry.addData("Direction", NewShooterController.reversed ? "REVERSED" : "FORWARD");
            telemetry.addData("Motor 2 Reversed", NewShooterController.secondMotorReversed ? "YES" : "NO");
            telemetry.addLine();

            // PID tuning info
            try {
                telemetry.addData("Integral Sum", "%.1f", shooter.getIntegralSum());
            } catch (Exception e) {
                // Method may not exist in original version
            }

            // Runtime
            telemetry.addLine();
            telemetry.addData("Runtime", "%.1f sec", runtime.seconds());

            // Updated controls reminder
            telemetry.addLine();
            telemetry.addLine("───────────────────────────────");
            telemetry.addLine("UP/DOWN=±50 | LEFT/RIGHT=Toggle Motor");
            telemetry.addLine("Y=Start | X=Stop | B=Direction");

            telemetry.update();
        }

        // Stop shooter when OpMode ends
        shooter.stopShooting();
    }

    /**
     * Creates a visual progress bar showing current vs target RPM
     */
    private String createProgressBar(double current, double target, double max) {
        int barLength = 20;
        int currentPos = (int) ((current / max) * barLength);
        int targetPos = (int) ((target / max) * barLength);

        currentPos = Math.max(0, Math.min(barLength, currentPos));
        targetPos = Math.max(0, Math.min(barLength, targetPos));

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < barLength; i++) {
            if (i == targetPos) {
                bar.append("|");  // Target marker
            } else if (i < currentPos) {
                bar.append("█");  // Filled
            } else {
                bar.append("░");  // Empty
            }
        }
        bar.append("]");

        return bar.toString();
    }
}