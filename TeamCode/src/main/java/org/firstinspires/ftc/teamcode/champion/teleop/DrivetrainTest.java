package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTest extends LinearOpMode {

    private SixWheelDriveController drive;

    @Override
    public void runOpMode() {
        // Initialize the drive controller
        drive = new SixWheelDriveController(this);

        telemetry.addLine("Drivetrain Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick Y: Forward/Backward");
        telemetry.addLine("  Right Stick X: Turn Left/Right");
        telemetry.addLine("  A Button: Toggle Speed Mode");
        telemetry.addLine("  B Button: Reset Odometry");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();

            // Get gamepad inputs
            double driveInput = -gamepad1.left_stick_y;  // Inverted for intuitive forward
            double turnInput = gamepad1.right_stick_x;

            // Apply speed mode multipliers
            double speedMultiplier = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_SPEED_MULTIPLIER :
                    SixWheelDriveController.SLOW_SPEED_MULTIPLIER;

            double turnMultiplier = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_TURN_MULTIPLIER :
                    SixWheelDriveController.SLOW_TURN_MULTIPLIER;

            // Scale inputs
            double drive_scaled = driveInput * speedMultiplier;
            double turn_scaled = turnInput * turnMultiplier;

            // Drive using velocity control
            drive.tankDriveVelocityNormalized(
                    drive_scaled + turn_scaled,
                    drive_scaled - turn_scaled
            );

            // Toggle speed mode with A button
            if (gamepad1.a) {
                drive.toggleSpeedMode();
                sleep(200);  // Debounce
            }

            // Reset odometry with B button
            if (gamepad1.b) {
                drive.resetOdometry();
                sleep(200);  // Debounce
            }

            // Display telemetry
            telemetry.addData("Speed Mode", drive.isFastSpeedMode() ? "FAST" : "SLOW");
            telemetry.addData("Drive Input", "%.2f", driveInput);
            telemetry.addData("Turn Input", "%.2f", turnInput);
            telemetry.addLine();

            telemetry.addData("Position X", "%.1f cm", drive.getX());
            telemetry.addData("Position Y", "%.1f cm", drive.getY());
            telemetry.addData("Heading", "%.1f°", drive.getHeadingDegrees());
            telemetry.addLine();

            telemetry.addData("Left Velocity", "%.0f ticks/sec", drive.getLeftVelocity());
            telemetry.addData("Right Velocity", "%.0f ticks/sec", drive.getRightVelocity());
            telemetry.addLine();

            drive.getMotorStatus();

            telemetry.update();
        }

        // Stop motors when OpMode ends
        drive.stopDrive();
    }
}
