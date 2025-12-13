package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.champion.controller.JohnController;

import java.util.List;

@TeleOp(name = "RPM Test with John Controller", group = "Test")
public class RPMTestOpmode extends LinearOpMode {

    private JohnController controller;

    @Override
    public void runOpMode() {
        // Initialize controller
        controller = new JohnController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Left Stick Y", "Drive Forward/Backward");
        telemetry.addData("Right Stick X", "Turn Left/Right");
        telemetry.addData("A Button", "Toggle Speed Mode");
        telemetry.addData("B Button", "Reset Odometry & RPM");
        telemetry.addData("X Button", "Show RPM History Info");
        telemetry.addData("Y Button", "Toggle Drive Mode");
        telemetry.update();

        waitForStart();

        boolean xButtonPreviouslyPressed = false;
        boolean yButtonPreviouslyPressed = false;
        boolean aButtonPreviouslyPressed = false;
        boolean bButtonPreviouslyPressed = false;

        while (opModeIsActive()) {
            // === CONTROL INPUT ===
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Apply speed multipliers
            if (controller.isFastSpeedMode()) {
                drive *= JohnController.FAST_SPEED_MULTIPLIER;
                turn *= JohnController.FAST_TURN_MULTIPLIER;
            } else {
                drive *= JohnController.SLOW_SPEED_MULTIPLIER;
                turn *= JohnController.SLOW_TURN_MULTIPLIER;
            }

            // Drive the robot
            controller.arcadeDrive(drive, turn);

            // === BUTTON CONTROLS ===

            // A Button - Toggle Speed Mode
            if (gamepad1.a && !aButtonPreviouslyPressed) {
                controller.toggleSpeedMode();
            }
            aButtonPreviouslyPressed = gamepad1.a;

            // B Button - Reset Everything
            if (gamepad1.b && !bButtonPreviouslyPressed) {
                controller.resetOdometry();
                telemetry.speak("Odometry and RPM reset");
            }
            bButtonPreviouslyPressed = gamepad1.b;

            // Y Button - Toggle Drive Mode
            if (gamepad1.y && !yButtonPreviouslyPressed) {
                if (controller.getCurrentDriveMode() == JohnController.DriveMode.VELOCITY) {
                    controller.setDriveMode(JohnController.DriveMode.POWER);
                } else {
                    controller.setDriveMode(JohnController.DriveMode.VELOCITY);
                }
            }
            yButtonPreviouslyPressed = gamepad1.y;

            // === UPDATE SYSTEMS ===
            controller.updateOdometry();
            controller.updateRPM();  // Update RPM readings and history

            // === TELEMETRY ===
            telemetry.addLine("=== RPM TEST MODE ===");
            telemetry.addLine();

            // Show motor status (includes RPM)
            controller.getMotorStatus();

            // X Button - Show detailed RPM history info
            if (gamepad1.x) {
                telemetry.addLine();
                telemetry.addLine("=== RPM HISTORY INFO ===");

                List<JohnController.RPMDataPoint> l1History = controller.getMotor1LeftHistory();
                List<JohnController.RPMDataPoint> r1History = controller.getMotor1RightHistory();

                telemetry.addData("History Size", "%d / %d points",
                        l1History.size(), JohnController.MAX_DATA_POINTS);

                if (l1History.size() > 0) {
                    JohnController.RPMDataPoint oldest = l1History.get(0);
                    JohnController.RPMDataPoint newest = l1History.get(l1History.size() - 1);
                    long timeSpan = newest.timestamp - oldest.timestamp;

                    telemetry.addData("Time Span", "%.1f seconds", timeSpan / 1000.0);
                    telemetry.addData("L1 Range", "%.1f to %.1f RPM",
                            getMinRPM(l1History), getMaxRPM(l1History));
                    telemetry.addData("R1 Range", "%.1f to %.1f RPM",
                            getMinRPM(r1History), getMaxRPM(r1History));
                }
            }

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addData("Speed Mode", controller.isFastSpeedMode() ? "FAST" : "SLOW");
            telemetry.addData("Drive Mode", controller.getCurrentDriveMode());
            telemetry.addData("Drive Input", "%.2f", drive);
            telemetry.addData("Turn Input", "%.2f", turn);

            telemetry.update();
        }

        // Stop motors when done
        controller.stopDrive();
    }

    // Helper methods for RPM history analysis
    private double getMinRPM(List<JohnController.RPMDataPoint> history) {
        if (history.isEmpty()) return 0.0;
        double min = Double.MAX_VALUE;
        for (JohnController.RPMDataPoint point : history) {
            if (point.rpm < min) min = point.rpm;
        }
        return min;
    }

    private double getMaxRPM(List<JohnController.RPMDataPoint> history) {
        if (history.isEmpty()) return 0.0;
        double max = Double.MIN_VALUE;
        for (JohnController.RPMDataPoint point : history) {
            if (point.rpm > max) max = point.rpm;
        }
        return max;
    }
}