package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

/**
 * RoadRunner Autonomous OpMode for FTC robot.
 * This OpMode demonstrates basic RoadRunner functionality with tank drive,
 * moving the robot forward and then returning to the starting position.
 */
@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {

    // Configurable constants for movement parameters
    public static final double FORWARD_DISTANCE_INCHES = 24.0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize starting pose at origin
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Initialize the drive system
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, startPose, telemetry);

        // Build the autonomous action: move forward to FORWARD_DISTANCE_INCHES, then back to origin
        Action movementAction = drive.actionBuilder(startPose)
                .setTangent(0) // Define forward direction (along +X)
                .lineToX(FORWARD_DISTANCE_INCHES) // Move forward while keeping same heading
                .lineToX(0) // Return to starting X position
                .build();

        // Display pre-start information
        displayPreStartTelemetry(drive, startPose);
        waitForStart();
        if (isStopRequested()) return;


        // Show "Running..." during execution
        telemetry.clear();
        telemetry.addLine("====================================");
        telemetry.addLine("EXECUTING AUTONOMOUS...");
        telemetry.addLine("====================================");
        telemetry.update();

        // Execute the autonomous action with real-time telemetry
        executeActionWithTelemetry(drive, movementAction);

        // Display final results
        displayFinalTelemetry(drive, startPose);

        // Keep telemetry visible until OpMode ends
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Displays pre-start telemetry information including starting position and planned movements.
     */
    private void displayPreStartTelemetry(AutoTankDrive drive, Pose2d startPose) {
        telemetry.addLine("====================================");
        telemetry.addLine("ROADRUNNER AUTONOMOUS");
        telemetry.addLine("====================================");
        telemetry.addLine("");
        telemetry.addLine("STARTING POSITION:");
        telemetry.addData("Status", drive.getDeviceStatus());
        telemetry.addData("  X", "%.2f inches", startPose.position.x);
        telemetry.addData("  Y", "%.2f inches", startPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addLine("");
        telemetry.addLine("PLANNED MOVEMENTS:");
        telemetry.addData("  1. Forward", "%.1f inches", FORWARD_DISTANCE_INCHES);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();
    }

    /**
     * Executes the given action with real-time telemetry updates.
     */
    private void executeActionWithTelemetry(AutoTankDrive drive, Action action) {
        while (!isStopRequested()) {
            boolean actionRunning = action.run(null);
            if (!actionRunning) {
                break;
            }

            updateExecutionTelemetry(drive);
            sleep(50); // Update rate
        }
    }

    /**
     * Updates telemetry during action execution with detailed drive information.
     */
    private void updateExecutionTelemetry(AutoTankDrive drive) {
        telemetry.clear();
        telemetry.addLine("====================================");
        telemetry.addLine("ROADRUNNER AUTONOMOUS - EXECUTING");
        telemetry.addLine("====================================");
        telemetry.addLine("");

        // Ramsete controller parameters
        telemetry.addLine("RAMSETE CONTROLLER:");
        telemetry.addData("  Zeta (damping)", "%.3f", drive.ramseteZeta);
        telemetry.addData("  BBar (aggressiveness)", "%.3f", drive.ramseteBBar);
        telemetry.addLine("");

        // Pose errors
        telemetry.addLine("POSE ERRORS:");
        telemetry.addData("  X Error", "%.2f inches", drive.xError);
        telemetry.addData("  Y Error", "%.2f inches", drive.yError);
        telemetry.addData("  Heading Error", "%.1f degrees", drive.headingErrorDeg);
        telemetry.addLine("");

        // Current pose
        telemetry.addLine("CURRENT POSITION:");
        telemetry.addData("  X", "%.2f inches", drive.currentX);
        telemetry.addData("  Y", "%.2f inches", drive.currentY);
        telemetry.addData("  Heading", "%.1f degrees", drive.currentHeadingDeg);
        telemetry.addLine("");

        // Target pose
        telemetry.addLine("TARGET POSITION:");
        telemetry.addData("  X", "%.2f inches", drive.targetX);
        telemetry.addData("  Y", "%.2f inches", drive.targetY);
        telemetry.addData("  Heading", "%.1f degrees", drive.targetHeadingDeg);
        telemetry.addLine("");

        // Velocities
        telemetry.addLine("VELOCITIES:");
        telemetry.addData("  Commanded Linear", "%.2f in/s", drive.commandLinVel);
        telemetry.addData("  Commanded Angular", "%.1f deg/s", drive.commandAngVelDeg);
        telemetry.addData("  Actual Linear", "%.2f in/s", drive.actualLinVel);
        telemetry.addData("  Actual Angular", "%.1f deg/s", drive.actualAngVelDeg);
        telemetry.addLine("");

        // Motor powers
        telemetry.addLine("MOTOR POWERS:");
        telemetry.addData("  Left Power", "%.3f", drive.leftPower);
        telemetry.addData("  Right Power", "%.3f", drive.rightPower);
        telemetry.addData("  Left Wheel Vel", "%.2f", drive.leftWheelVel);
        telemetry.addData("  Right Wheel Vel", "%.2f", drive.rightWheelVel);
        telemetry.addLine("");

        // Additional telemetry
        telemetry.addLine("ADDITIONAL DATA:");
        telemetry.addData("  Elapsed Time", "%.2f seconds", drive.elapsedTime);
        telemetry.addData("  Battery Voltage", "%.2f V", drive.batteryVoltage);

        telemetry.update();
    }

    /**
     * Displays final telemetry after autonomous completion.
     */
    private void displayFinalTelemetry(AutoTankDrive drive, Pose2d startPose) {
        // Update pose estimate one final time
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.pinpointLocalizer.getPose();

        telemetry.clear();
        telemetry.addLine("====================================");
        telemetry.addLine("AUTONOMOUS COMPLETE!");
        telemetry.addLine("====================================");
        telemetry.addLine("");

        telemetry.addLine("STARTING POSITION:");
        telemetry.addData("  X", "%.2f inches", startPose.position.x);
        telemetry.addData("  Y", "%.2f inches", startPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addLine("");

        telemetry.addLine("FINAL POSITION:");
        telemetry.addData("  X", "%.2f inches", finalPose.position.x);
        telemetry.addData("  Y", "%.2f inches", finalPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addLine("");
    }
}