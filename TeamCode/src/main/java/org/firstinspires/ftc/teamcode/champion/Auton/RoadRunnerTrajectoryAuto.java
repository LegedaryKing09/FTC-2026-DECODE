package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.TankDrive;

@Config
@Autonomous(name = "RoadRunner Trajectory Auto")
public class RoadRunnerTrajectoryAuto extends LinearOpMode {

    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING_DEG = 0.0;

    public static double TARGET_X = 24.0;
    public static double TARGET_Y = 36.0;
    public static double TARGET_HEADING_DEG = 90.0;
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));

        TankDrive drive = new TankDrive(hardwareMap, startPose);

        Action trajectoryToTarget = drive.actionBuilder(startPose)
                .splineToLinearHeading(
                        new Pose2d(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING_DEG)),
                        Math.toRadians(TARGET_HEADING_DEG)
                )
                .build();

        Action moveToPositionThenTurn = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(TARGET_X, TARGET_Y), Math.toRadians(TARGET_HEADING_DEG))
                .turn(Math.toRadians(TARGET_HEADING_DEG - START_HEADING_DEG))
                .build();

        Action stepByStepTrajectory = drive.actionBuilder(startPose)
                .lineToY(TARGET_Y / 2)
                .lineToX(TARGET_X)
                .lineToY(TARGET_Y)
                .turnTo(Math.toRadians(TARGET_HEADING_DEG))
                .build();

        Action complexTrajectory = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(12, 12), Math.toRadians(45))
                .splineTo(new Vector2d(TARGET_X, TARGET_Y), Math.toRadians(TARGET_HEADING_DEG))
                .turnTo(Math.toRadians(TARGET_HEADING_DEG))
                .build();

        telemetry.addData("Start Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Target Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                TARGET_X, TARGET_Y, TARGET_HEADING_DEG);
        telemetry.addLine();
        telemetry.addLine("Ready to run trajectory!");
        telemetry.addLine("Press START to begin autonomous");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            telemetry.addLine("Running direct trajectory to target...");
            telemetry.update();

            Actions.runBlocking(trajectoryToTarget);

            sleep(500);

            telemetry.addLine("Trajectory complete!");
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                    drive.localizer.getPose().position.x,
                    drive.localizer.getPose().position.y,
                    Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
    }
}