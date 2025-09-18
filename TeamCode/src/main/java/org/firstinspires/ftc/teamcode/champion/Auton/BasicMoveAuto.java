package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.RoadRunnerDrive;

@Autonomous
public class BasicMoveAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(0,0, 0);
    Pose2d forwardPose = new Pose2d(20,0, 0);
    Pose2d endPose = new Pose2d(20,20, 90);


    @Override
    public void runOpMode() {

        RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(forwardPose)
                .build();

        Trajectory turnforward = drive.trajectoryBuilder(forwardPose)
                .lineToLinearHeading(endPose)
                .build();

        waitForStart();

        drive.followTrajectory(forward);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(turnforward);

    }

}
