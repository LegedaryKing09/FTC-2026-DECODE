package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp(name = "Odometry Test Teleop", group = "Teleop")
public class OdometryTestTeleop extends LinearOpMode {

    private SixWheelDriveController driveController;
    private boolean lastBackButton = false;

    @Override
    public void runOpMode() {
        driveController = new SixWheelDriveController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press BACK to toggle speed mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            driveController.updateOdometry();

            // Simple tank drive controls
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Apply speed multipliers
            if (driveController.isFastSpeedMode()) {
                drive *= SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn *= SixWheelDriveController.FAST_TURN_MULTIPLIER;
            } else {
                drive *= SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                turn *= SixWheelDriveController.SLOW_TURN_MULTIPLIER;
            }

            driveController.arcadeDrive(drive, turn);

            // Toggle speed mode with BACK button
            if (gamepad1.back && !lastBackButton) {
                driveController.toggleSpeedMode();
            }
            lastBackButton = gamepad1.back;

            // Reset odometry with START button
            if (gamepad1.start) {
                driveController.resetOdometry();
                sleep(200); // Debounce
            }

            // Display odometry data
            telemetry.addData("Speed Mode", driveController.isFastSpeedMode() ? "FAST" : "SLOW");
            telemetry.addLine();

            telemetry.addData("Position X (inches)", "%.2f", driveController.getX());
            telemetry.addData("Position Y (inches)", "%.2f", driveController.getY());
            telemetry.addData("Heading (degrees)", "%.2f", driveController.getHeadingDegrees());
            telemetry.addLine();

            telemetry.addData("Raw Encoder X", driveController.getXOdoPosition());
            telemetry.addData("Raw Encoder Y", driveController.getYOdoPosition());
            telemetry.addLine();

            // Debug raw pinpoint values
            Pose2D rawPose = driveController.getPinpoint().getPosition();
            telemetry.addData("Raw Pinpoint X (mm)", "%.2f", rawPose.getX(DistanceUnit.MM));
            telemetry.addData("Raw Pinpoint Y (mm)", "%.2f", rawPose.getY(DistanceUnit.MM));
            telemetry.addData("Raw Pinpoint X (in)", "%.2f", rawPose.getX(DistanceUnit.INCH));
            telemetry.addData("Raw Pinpoint Y (in)", "%.2f", rawPose.getY(DistanceUnit.INCH));
            telemetry.addData("Raw Pinpoint Heading", "%.2f°", Math.toDegrees(rawPose.getHeading(AngleUnit.RADIANS)));
            telemetry.addLine();

            telemetry.addData("Velocity X (in/s)", "%.2f", driveController.getVelocityX());
            telemetry.addData("Velocity Y (in/s)", "%.2f", driveController.getVelocityY());
            telemetry.addData("Heading Velocity (rad/s)", "%.4f", driveController.getHeadingVelocity());
            telemetry.addLine();

            telemetry.addData("Pinpoint Status", driveController.getPinpointStatus());
            telemetry.addData("Pinpoint Loop Time (μs)", driveController.getPinpointLoopTime());
            telemetry.addData("Pinpoint Frequency (Hz)", "%.1f", driveController.getPinpointFrequency());
            telemetry.addLine();

            telemetry.addData("Motor Powers", driveController.getMotorPowers());

            telemetry.update();
        }

        driveController.stopDrive();
    }
}