package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.RoadRunnerDrive;

@Autonomous
public class BasicMoveAuto extends LinearOpMode {
    RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap);
    Pose2d startPose = new Pose2d(0,0, 0);


    @Override
    public void runOpMode() {

        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(20)
                .build();

        drive.followTrajectory(forward);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forward);

    }

}
