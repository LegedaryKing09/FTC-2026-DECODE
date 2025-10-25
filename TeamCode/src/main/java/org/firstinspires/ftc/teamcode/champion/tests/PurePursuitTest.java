package org.firstinspires.ftc.teamcode.champion.tests;

import android.annotation.SuppressLint;

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

    @SuppressLint("DefaultLocale")
    public void goToPosition(double x, double y) {
        Vector2d targetPosition = new Vector2d(x, y);
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
    /**
     * Rotates the robot to the specified heading angle.
     * This method blocks until the target heading is reached.
     *
     * @param targetHeadingDeg Target heading in degrees (0 = forward, 90 = left, -90 = right)
     */
    public void turnToHeading(double targetHeadingDeg) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        double HEADING_THRESHOLD = Math.toRadians(2.0); // 2 degrees tolerance
        double TURN_POWER = 0.3; // Adjust this for turn speed

        telemetry.addData("Target Heading", "%.1f deg", targetHeadingDeg);
        telemetry.update();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = targetHeadingRad - currentHeading;

            // Normalize error to [-PI, PI]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Check if we've reached the target
            if (Math.abs(headingError) < HEADING_THRESHOLD) {
                driveController.stopDrive();
                telemetry.addData("Status", "Turn Complete");
                telemetry.update();
                break;
            }

            // Calculate turn power (positive = counterclockwise, negative = clockwise)
            double turnPower = Math.signum(headingError) * TURN_POWER;
            driveController.tankDrive(-turnPower, turnPower); // Left negative, right positive = turn right

            telemetry.addData("Status", "Turning");
            telemetry.addData("Current Heading", "%.1f deg", Math.toDegrees(currentHeading));
            telemetry.addData("Heading Error", "%.1f deg", Math.toDegrees(headingError));
            telemetry.update();
        }
    }
}