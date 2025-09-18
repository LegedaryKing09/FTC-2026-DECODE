package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.RoadRunnerDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Move forward 24 inches
        Trajectory moveForward = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();
        drive.followTrajectory(moveForward);

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();
        }

        //turn left 90 degree
        drive.turn(Math.toRadians(90));

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();
        }

        Pose2d CurrentPose = drive.getPoseEstimate();

        // Move backward 12 inches
        Trajectory moveBack = drive.trajectoryBuilder(CurrentPose)
                .back(12)
                .build();
        drive.followTrajectory(moveBack);

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();
        }

        // Stop motors
        drive.stopDrive();
    }
}



