package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {

    public static double ForwardInch = 24.0;
    public static double SecondTurn = 180.0;


    @Override
    public void runOpMode() throws InterruptedException {
        // starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        AutoTankDrive drive = new AutoTankDrive(hardwareMap, startPose);

        drive.initializePinpoint(this,startPose);

        Action moveForwardAndTurn = drive.actionBuilder(startPose)
                .setTangent(0) // define forward direction (along +X)
                .lineToXConstantHeading(ForwardInch) // move forward while keeping same heading
                .turn(Math.toRadians(SecondTurn)) // then turn left 180°
                .build();

        // Display starting position before start
        telemetry.addLine("====================================");
        telemetry.addLine("ROADRUNNER AUTONOMOUS");
        telemetry.addLine("====================================");
        telemetry.addLine("");
        telemetry.addLine("STARTING POSITION:");
        telemetry.addData("  X", "%.2f inches", startPose.position.x);
        telemetry.addData("  Y", "%.2f inches", startPose.position.y);
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.addLine("");
        telemetry.addLine("PLANNED MOVEMENTS:");
        telemetry.addData("  1. Forward", "%.1f inches", ForwardInch);
        telemetry.addData("  2. Turn", "%.1f degrees", SecondTurn);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;


        // Show "Running..." during execution
        telemetry.clear();
        telemetry.addLine("====================================");
        telemetry.addLine("EXECUTING AUTONOMOUS...");
        telemetry.addLine("====================================");
        telemetry.update();

        Actions.runBlocking(moveForwardAndTurn);

        // Update pose estimate one final time
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.localizer.getPose();

        // Display final results
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


        // Keep telemetry visible
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            sleep(50);
    }
}
}