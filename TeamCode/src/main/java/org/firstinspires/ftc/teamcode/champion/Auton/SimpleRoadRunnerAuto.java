package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.SimpleTankDrive;

@Config
@Autonomous(name = "Simple RoadRunner Auto")
public class SimpleRoadRunnerAuto extends LinearOpMode {

    // Tunable via FTC Dashboard
    public static double TARGET_X = 48.0;           // inches forward
    public static double TARGET_Y = 0.0;           // inches left
    public static double TARGET_HEADING = 0.0;     // degrees

    public static double SECOND_X = 0.0;
    public static double SECOND_Y = 0.0;
    public static double SECOND_HEADING = 0;    // degrees

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive
        SimpleTankDrive drive = new SimpleTankDrive(hardwareMap, this);

        // Wait for Pinpoint to be ready
        drive.waitForPinpointReady();

        // Set starting position
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPose(startPose);

        telemetry.addData("Start Pose", "(%.1f, %.1f) @ %.1f°",
            startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addData("Target 1", "(%.1f, %.1f)", TARGET_X, TARGET_Y);
        telemetry.addData("Target 2 Heading", "%.1f°", SECOND_HEADING);
        telemetry.addData("Status", "Ready - Press START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ============================================
        // AUTONOMOUS SEQUENCE
        // ============================================

        telemetry.clear();
        telemetry.addData("Status", "Executing autonomous...");
        telemetry.update();

        // Movement 1: Drive to target position (no heading change)
        telemetry.addLine("");
        telemetry.addLine("Movement 1: Driving to target...");
        telemetry.update();

        Action move1 = drive.driveTo(TARGET_X, TARGET_Y);
        Actions.runBlocking(move1);

        // Brief pause to verify final position
        sleep(500);

        // Update odometry to get accurate final position
        drive.updateOdometry();
        Pose2d pose1 = drive.getPose();

        telemetry.addLine("");
        telemetry.addLine("Movement 1 Complete:");
        telemetry.addData("  Final X", "%.2f inches", pose1.position.x);
        telemetry.addData("  Final Y", "%.2f inches", pose1.position.y);
        telemetry.addData("  Error X", "%.2f inches", TARGET_X - pose1.position.x);
        telemetry.addData("  Error Y", "%.2f inches", TARGET_Y - pose1.position.y);
        telemetry.update();

        sleep(500);  // Pause between movements

        // Movement 2: Turn to target heading
        telemetry.addLine("");
        telemetry.addLine("Movement 2: Turning to target heading...");
        telemetry.update();

        Action move2 = drive.turnTo(SECOND_HEADING);
        Actions.runBlocking(move2);

        // Brief pause to verify final heading
        sleep(500);

        // Update odometry to get accurate final heading
        drive.updateOdometry();
        Pose2d pose2 = drive.getPose();

        telemetry.addLine("");
        telemetry.addLine("Movement 2 Complete:");
        telemetry.addData("  Final Heading", "%.2f degrees", Math.toDegrees(pose2.heading.toDouble()));
        telemetry.addData("  Heading Error", "%.2f degrees",
                SECOND_HEADING - Math.toDegrees(pose2.heading.toDouble()));
        telemetry.update();

        // Optional: Movement 3 - Drive to another position with heading
        // Uncomment if you want to test driveToPose
        // telemetry.addLine("");
        // telemetry.addLine("Movement 3: Drive to pose...");
        // telemetry.update();
        //
        // Action move3 = drive.driveToPose(SECOND_X, SECOND_Y, SECOND_HEADING);
        // Actions.runBlocking(move3);

        // ============================================
        // DISPLAY FINAL RESULTS
        // ============================================

        // Update odometry one final time to ensure accuracy
        drive.updateOdometry();
        Pose2d finalPose = drive.getPose();

        telemetry.clear();
        telemetry.addData("Result", "Autonomous Complete");

        double errorX = TARGET_X - finalPose.position.x;
        double errorY = TARGET_Y - finalPose.position.y;
        double errorHeading = SECOND_HEADING - Math.toDegrees(finalPose.heading.toDouble());
        double totalError = Math.sqrt(errorX * errorX + errorY * errorY);

        telemetry.addData("Final Position", "(%.2f, %.2f) @ %.1f°",
            finalPose.position.x, finalPose.position.y, Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addData("Position Error", "X:%.2f Y:%.2f Total:%.2f in", errorX, errorY, totalError);
        telemetry.addData("Heading Error", "%.1f°", errorHeading);
        telemetry.addData("Accuracy", totalError < 1.0 ? "EXCELLENT" : "NEEDS TUNING");
        telemetry.update();

        // Keep telemetry visible - continuously update to show live values
        while (opModeIsActive() && !isStopRequested()) {
            // Update odometry to show live position if robot is moved
            drive.updateOdometry();
            Pose2d livePose = drive.getPose();

            // Show live position at bottom of screen
            telemetry.addLine("");
            telemetry.addLine("--- LIVE POSITION ---");
            telemetry.addData("Live X", "%.2f inches", livePose.position.x);
            telemetry.addData("Live Y", "%.2f inches", livePose.position.y);
            telemetry.addData("Live Heading", "%.1f degrees",
                    Math.toDegrees(livePose.heading.toDouble()));
            telemetry.update();
            sleep(100);
        }
    }
}