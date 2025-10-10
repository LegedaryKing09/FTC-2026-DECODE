package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.PurePursuitController;


@TeleOp(name = "Pure Pursuit Test", group = "Tests")
public class PurePursuitTest extends OpMode {

    private SixWheelDriveController driveController;
    private PurePursuitController pursuitController;
    private Pose2d targetPose;

    @Override
    public void init() {
        // Initialize drive controller
        driveController = new SixWheelDriveController(this);

        // Initialize pure pursuit controller
        pursuitController = new PurePursuitController();
        pursuitController.setParameters(12.0, 0.5, 12.0); // look ahead, max speed, track width

        // Set target pose: go to (48,48) with heading 0 (straight)
        targetPose = new Pose2d(48, 48, 0);
        pursuitController.setTargetPose(targetPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)", targetPose.position.x, targetPose.position.y, Math.toDegrees(targetPose.heading.toDouble())));
    }

    @Override
    public void loop() {
        // Update odometry
        driveController.updateOdometry();

        // Get current pose
        Pose2d currentPose = new Pose2d(
            driveController.getX(),
            driveController.getY(),
            driveController.getHeading()
        );

        // Update pure pursuit
        double[] powers = pursuitController.update(currentPose);
        driveController.tankDrive(powers[0], powers[1]);

        // Check if at end
        Vector2d endPoint = targetPose.position;
        double distToEnd = Math.hypot(currentPose.position.x - endPoint.x, currentPose.position.y - endPoint.y);
        if (distToEnd < 2.0) {
            driveController.stopDrive();
            telemetry.addData("Status", "Path Complete");
        } else {
            telemetry.addData("Status", "Following Path");
        }

        // Telemetry
        telemetry.addData("X", "%.1f", currentPose.position.x);
        telemetry.addData("Y", "%.1f", currentPose.position.y);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addData("Left Power", "%.2f", powers[0]);
        telemetry.addData("Right Power", "%.2f", powers[1]);
        telemetry.addData("Dist to End", "%.1f", distToEnd);
    }

    @Override
    public void stop() {
        driveController.stopDrive();
    }
}