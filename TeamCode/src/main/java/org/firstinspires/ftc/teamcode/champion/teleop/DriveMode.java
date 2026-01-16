package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp(name = "Drive Mode Test", group = "Test")
public class DriveMode extends LinearOpMode {

    private SixWheelDriveController driveController;
    private boolean velocityMode = true; // Start in velocity mode
    private boolean lastAPress = false;

    @Override
    public void runOpMode() {
        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        telemetry.addLine("Drive Mode Test");
        telemetry.addLine("Press A to toggle between modes");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle drive mode with A button (single press detection)
            boolean currentAPress = gamepad1.a;
            if (currentAPress && !lastAPress) {
                velocityMode = !velocityMode;

                if (velocityMode) {
                    driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                } else {
                    driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);
                }
            }
            lastAPress = currentAPress;

            // Get joystick inputs
            double leftY = -gamepad1.left_stick_y;
            double rightY = -gamepad1.right_stick_y;

            // Drive based on current mode
            if (velocityMode) {
                // Velocity mode: normalized input (0.0 to 1.0)
                driveController.tankDriveVelocityNormalized(leftY, rightY);
            } else {
                // Power mode: direct power
                driveController.tankDrive(leftY, rightY);
            }

            // Update odometry
            driveController.updateOdometry();

            // Telemetry
            telemetry.addLine("=== DRIVE MODE TEST ===");
            telemetry.addData("Mode", velocityMode ? "VELOCITY (Encoder)" : "POWER (Direct)");
            telemetry.addLine();
            telemetry.addData("Left Stick", "%.2f", leftY);
            telemetry.addData("Right Stick", "%.2f", rightY);
            telemetry.addLine();
            telemetry.addData("X Position", "%.1f inches", driveController.getX());
            telemetry.addData("Y Position", "%.1f inches", driveController.getY());
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(driveController.getHeading()));
            telemetry.addLine();
            telemetry.addLine("Press A to toggle mode");
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();
    }
}