package org.firstinspires.ftc.teamcode.champion.Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.RoadRunnerDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode{
        @Override
        public void runOpMode(){
            RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap);
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            telemetry.addData("Status", "Ready to run");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            // Move forward 24 inches
            Trajectory moveForward = drive.trajectoryBuilder(startPose)
                    .forward(24)
                    .build();

            drive.followTrajectory(moveForward);
            while (drive.isBusy() && !isStopRequested()) {
                drive.update();
                Pose2d currentPose = drive.getPoseEstimate();
                telemetry.addData("Status", "Moving Forward");
                telemetry.addData("x", currentPose.getX());
                telemetry.addData("y", currentPose.getY());
                telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
                telemetry.update();
            }

            // Turn left 90 degrees
            Trajectory turnLeft = drive.trajectoryBuilder(moveForward.end())
                    .splineToSplineHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(90))
                    .build();

            drive.followTrajectory(turnLeft);

            while (drive.isBusy() && !isStopRequested()) {
                drive.update();
                telemetry.addData("Status", "Turning Left");
                telemetry.update();
            }

            // Move backward 12 inches
            Trajectory moveBack = drive.trajectoryBuilder(turnLeft.end())
                    .back(12)
                    .build();

            drive.followTrajectory(moveBack);

            while (drive.isBusy() && !isStopRequested()) {
                drive.update();
                telemetry.addData("Status", "Moving Back");
                telemetry.update();
            }

            telemetry.addData("Status", "Complete!");
            telemetry.update();

            // Stop motors
            drive.stop();
        }

}



