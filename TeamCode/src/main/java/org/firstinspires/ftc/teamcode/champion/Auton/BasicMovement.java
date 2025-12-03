package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@Autonomous(name = "Basic Movement", group = "Competition")
public class BasicMovement extends LinearOpMode {
    // Controllers
    SixWheelDriveController driveController;

    public static double MOVEMENT_SPEED = 0.5;
    public static double TARGET_DISTANCE = 72.0; // inches
    public static double POSITION_TOLERANCE = 1.0; // Stop when within 1 inch

    private final ElapsedTime globalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeRobot();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        cleanup();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        // Set to POWER mode (no velocity control)
        driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);
        // Reset odometry to start at 0,0
        driveController.resetOdometry();
    }

    private void executeAutonomousSequence() {
        // Record starting position
        double startX = driveController.getX();

        telemetry.addData("Status", "Moving forward...");
        telemetry.addData("Start X", startX);
        telemetry.addData("Target Distance", TARGET_DISTANCE);
        telemetry.update();

        // Loop until we've traveled 72 inches
        while (opModeIsActive()) {
            // Update odometry to get current position
            driveController.updateOdometry();

            double currentX = driveController.getX();
            double distanceTraveled = currentX - startX;

            // Check if we've reached the target
            if (distanceTraveled >= TARGET_DISTANCE - POSITION_TOLERANCE) {
                break;
            }

            // Move forward
            driveController.tankDrive(MOVEMENT_SPEED, MOVEMENT_SPEED);

            // Display telemetry
            telemetry.addData("Current X", currentX);
            telemetry.addData("Distance Traveled", distanceTraveled);
            telemetry.addData("Remaining", TARGET_DISTANCE - distanceTraveled);
            telemetry.update();

            // Small sleep to prevent loop from running too fast
            sleep(10);
        }

        // Stop the robot
        driveController.stopDrive();

        // Final telemetry
        telemetry.addData("Status", "Target Reached!");
        telemetry.addData("Final Distance", driveController.getX() - startX);
        telemetry.update();
        sleep(1000); // Hold final message for 1 second
    }

    private void cleanup() {
        driveController.stopDrive();
    }
}