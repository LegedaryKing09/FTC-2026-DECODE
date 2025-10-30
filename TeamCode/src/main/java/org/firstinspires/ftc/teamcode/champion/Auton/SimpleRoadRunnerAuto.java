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

        // Display startup info
        telemetry.addLine("====================================");
        telemetry.addLine("SIMPLE ROADRUNNER AUTONOMOUS");
        telemetry.addLine("====================================");
        telemetry.addLine("");
        telemetry.addLine("STARTING POSITION:");
        telemetry.addData("  X", "%.2f inches", startPose.position.x);
        telemetry.addData("  Y", "%.2f inches", startPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addLine("");
        telemetry.addLine("PLANNED MOVEMENTS:");
        telemetry.addData("  1. Drive to", "(%.1f, %.1f)", TARGET_X, TARGET_Y);
        telemetry.addData("  2. Turn to", "%.1f degrees", SECOND_HEADING);
        telemetry.addLine("");
        drive.displayTuningParams();
        telemetry.addLine("");
        telemetry.addLine("PURE RAMSETE CONTROL:");
        telemetry.addLine("No fixed STOP_DISTANCE needed!");
        telemetry.addLine("Velocity automatically scales");
        telemetry.addLine("with distance to target");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ============================================
        // AUTONOMOUS SEQUENCE
        // ============================================

        telemetry.clear();
        telemetry.addLine("====================================");
        telemetry.addLine("EXECUTING AUTONOMOUS...");
        telemetry.addLine("====================================");
        telemetry.addLine("");
        telemetry.addLine("Watch both Driver Station AND");
        telemetry.addLine("FTC Dashboard - values should match!");
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
        telemetry.addLine("====================================");
        telemetry.addLine("AUTONOMOUS COMPLETE!");
        telemetry.addLine("====================================");
        telemetry.addLine("");

        telemetry.addLine("STARTING POSITION:");
        telemetry.addData("  X", "%.2f inches", startPose.position.x);
        telemetry.addData("  Y", "%.2f inches", startPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addLine("");

        telemetry.addLine("TARGET POSITION:");
        telemetry.addData("  X", "%.2f inches", TARGET_X);
        telemetry.addData("  Y", "%.2f inches", TARGET_Y);
        telemetry.addData("  Heading", "%.1f degrees", SECOND_HEADING);
        telemetry.addLine("");

        telemetry.addLine("FINAL POSITION:");
        telemetry.addData("  X", "%.2f inches", finalPose.position.x);
        telemetry.addData("  Y", "%.2f inches", finalPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.addLine("");

        telemetry.addLine("ERRORS:");
        double errorX = TARGET_X - finalPose.position.x;
        double errorY = TARGET_Y - finalPose.position.y;
        double errorHeading = SECOND_HEADING - Math.toDegrees(finalPose.heading.toDouble());
        double totalError = Math.sqrt(errorX * errorX + errorY * errorY);

        telemetry.addData("  X Error", "%.2f inches", errorX);
        telemetry.addData("  Y Error", "%.2f inches", errorY);
        telemetry.addData("  Total Position Error", "%.2f inches", totalError);
        telemetry.addData("  Heading Error", "%.1f degrees", errorHeading);
        telemetry.addLine("");

        // Provide RAMSETE tuning feedback
        telemetry.addLine("RAMSETE TUNING GUIDE:");
        if (totalError > 1.0) {
            telemetry.addLine("    (Higher = more damping)");
        } else {
            telemetry.addLine("  ✓ Position accuracy is EXCELLENT!");
            telemetry.addLine("  ✓ RAMSETE parameters well tuned!");
            telemetry.addLine("");
            telemetry.addLine("Current settings:");
        }
        telemetry.addLine("");

        telemetry.addLine("HOW IT WORKS:");
        telemetry.addLine("• Velocity = Max × tanh(gain × distance)");
        telemetry.addLine("• Naturally slows as distance → 0");
        telemetry.addLine("• Works for ANY target distance!");
        telemetry.addLine("• No manual STOP_DISTANCE tuning");
        telemetry.addLine("");

        telemetry.addLine("VERIFY TELEMETRY SYNC:");
        telemetry.addLine("Check that Driver Station values");
        telemetry.addLine("match FTC Dashboard values!");
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