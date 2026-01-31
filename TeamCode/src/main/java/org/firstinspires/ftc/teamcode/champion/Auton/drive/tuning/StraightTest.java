package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
@Config
@Autonomous(name = "Straight Test", group = "Tuning")
public class StraightTest extends LinearOpMode {
    AutoTankDrive tankDrive;
    public static double FORWARD_DISTANCE = 24.0;
    public static double STARTING_DISTANCE = 0.0;

    @Override
    public void runOpMode() {
        waitForStart();

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        Forward();

        // Wait for settling
        sleep(500);

    }

    private void Forward() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        Action Forward = tankDrive.actionBuilder(currentPose)
                .lineToX(FORWARD_DISTANCE)
                .build();
        Actions.runBlocking(Forward);

        currentPose = tankDrive.pinpointLocalizer.getPose();

        Action Backward = tankDrive.actionBuilder(currentPose)
                .lineToX(STARTING_DISTANCE)
                .build();
        Actions.runBlocking(Backward);
    }




}
