package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp(name = "Odometry Test Teleop", group = "Teleop")
public class OdometryTestTeleop extends OpMode {

    private SixWheelDriveController driveController;

    @Override
    public void init() {
        driveController = new SixWheelDriveController(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry
        driveController.updateOdometry();

        // Simple tank drive controls
        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        if (driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.FAST_TURN_MULTIPLIER;
            }
            if (!driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;
            }

        driveController.arcadeDrive(drive, turn);

        // Display odometry data
        telemetry.addData("Position X (inches)", driveController.getX());
        telemetry.addData("Position Y (inches)", driveController.getY());
        telemetry.addData("Heading (degrees)", driveController.getHeadingDegrees());
        telemetry.addData("Raw Encoder X", driveController.getXOdoPosition());
        telemetry.addData("Raw Encoder Y", driveController.getYOdoPosition());
        telemetry.addData("Velocity X (mm/s)", driveController.getVelocityX());
        telemetry.addData("Velocity Y (mm/s)", driveController.getVelocityY());
        telemetry.addData("Heading Velocity (rad/s)", driveController.getHeadingVelocity());
        telemetry.addData("Pinpoint Status", driveController.getPinpointStatus());
        telemetry.addData("Pinpoint Loop Time (μs)", driveController.getPinpointLoopTime());
        telemetry.addData("Pinpoint Frequency (Hz)", driveController.getPinpointFrequency());
        telemetry.addData("Motor Powers", driveController.getMotorPowers());

        telemetry.update();
    }

    @Override
    public void stop() {
        driveController.stopDrive();
    }
}