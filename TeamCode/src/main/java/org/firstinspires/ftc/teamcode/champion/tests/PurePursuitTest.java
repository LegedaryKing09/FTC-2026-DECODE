package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.PurePursuitController;

@Config
@TeleOp(name = "Pure Pursuit Test", group = "Tests")
public class PurePursuitTest extends LinearOpMode {

    private SixWheelDriveController driveController;
    private PurePursuitController pursuitController;
    private Vector2d targetPosition;
    public static double targetX = 48;
    public static double targetY = 48;
    private static final double DISTANCE_THRESHOLD = 5.0;

    @Override
    public void runOpMode() {
        // Initialize drive controller
        driveController = new SixWheelDriveController(this);

        // Initialize pure pursuit controller
        pursuitController = new PurePursuitController();
        pursuitController.setParameters(12.0, 0.5, 15.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            goToPosition(targetX, targetY);
        }
    }

    public void goToPosition(double x, double y) {
        targetPosition = new Vector2d(x, y);
        pursuitController.setTargetPosition(targetPosition);

        telemetry.addData("Target", String.format("(%.1f, %.1f)", targetPosition.x, targetPosition.y));
        telemetry.update();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            Pose2d currentPose = new Pose2d(
                    driveController.getX(),
                    driveController.getY(),
                    driveController.getHeading()
            );

            double distToEnd = Math.hypot(
                    currentPose.position.x - targetPosition.x,
                    currentPose.position.y - targetPosition.y
            );

            if (distToEnd < DISTANCE_THRESHOLD) {
                driveController.stopDrive();
                telemetry.addData("Status", "Path Complete");
                telemetry.update();
                break;
            }

            double[] powers = pursuitController.update(currentPose);
            driveController.tankDrive(powers[0], powers[1]);

            telemetry.addData("Status", "Following Path");
            telemetry.addData("X", "%.1f", currentPose.position.x);
            telemetry.addData("Y", "%.1f", currentPose.position.y);
            telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Left Power", "%.2f", powers[0]);
            telemetry.addData("Right Power", "%.2f", powers[1]);
            telemetry.addData("Dist to End", "%.1f", distToEnd);
            telemetry.update();
        }
    }
}