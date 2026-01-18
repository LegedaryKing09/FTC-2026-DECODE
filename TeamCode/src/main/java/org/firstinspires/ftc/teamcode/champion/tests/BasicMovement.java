package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

@Config
@Autonomous(name = "Road Runner Movement Test", group = "Tuning")
public class BasicMovement extends LinearOpMode {
    public static double firstDistance = -65.0;
    public static double firstAngle = 90.0;
    public static double SecondDistance = 20.0;
    public static double SecondAngle = 180.0;
    public static double thirdDistance = 20.0;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, startPose);

        waitForStart();

        // Move backward 65 inches
        Action moveBack = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .lineToX(firstDistance)
                .build();
        Actions.runBlocking(moveBack);

        sleep(500);


        Pose2d currentPose = drive.pinpointLocalizer.getPose();
        drive.pinpointLocalizer.resetPinpoint(); // Reset to eliminate drift
        drive.pinpointLocalizer.setPose(currentPose); // Restore position (keeps x,y but resets heading reference)

        // First turn: 90 degrees
        Action turn90 = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .turn(Math.toRadians(90))
                .build();
        Actions.runBlocking(turn90);

        sleep(500);

        // Move forward then back
        Action moveForwardBack = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .lineToY(20)
                .lineToY(0)
                .build();
        Actions.runBlocking(moveForwardBack);

        sleep(500);

        currentPose = drive.pinpointLocalizer.getPose();
        drive.pinpointLocalizer.resetPinpoint();
        drive.pinpointLocalizer.setPose(currentPose);

        // Second turn: -90 degrees
        Action turnNeg90 = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .turn(Math.toRadians(-90))
                .build();
        Actions.runBlocking(turnNeg90);

        sleep(500);

        // Move forward
        Action moveForward = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .lineToX(-45)
                .build();
        Actions.runBlocking(moveForward);

        sleep(500);


        currentPose = drive.pinpointLocalizer.getPose();
        drive.pinpointLocalizer.resetPinpoint();
        drive.pinpointLocalizer.setPose(currentPose);

        // Third turn: 180 degrees
        Action turn180 = drive.actionBuilder(drive.pinpointLocalizer.getPose())
                .turn(Math.toRadians(180))
                .build();
        Actions.runBlocking(turn180);

        // Final status
        telemetry.addData("Status", "All movements complete!");
        telemetry.addData("Final Heading", Math.toDegrees(drive.pinpointLocalizer.getPose().heading.toDouble()));
        telemetry.addData("Final Position X", drive.pinpointLocalizer.getPose().position.x);
        telemetry.addData("Final Position Y", drive.pinpointLocalizer.getPose().position.y);
        telemetry.update();

        sleep(30000);
    }
}