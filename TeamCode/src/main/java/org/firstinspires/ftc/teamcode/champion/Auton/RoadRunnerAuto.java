package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

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
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, startPose);

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

    private void executeActionWithTelemetry(AutoTankDrive drive, Action action) {
        while (!isStopRequested()) {
            boolean actionRunning = action.run(null);
            if (!actionRunning) {
                break;
            }
            sleep(50); // Update rate
        }
    }

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