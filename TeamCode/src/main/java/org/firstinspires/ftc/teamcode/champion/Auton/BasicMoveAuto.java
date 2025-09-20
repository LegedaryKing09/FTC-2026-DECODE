package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.TankDrive;

@Autonomous
public class BasicMoveAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(0, 0, 0);
    Vector2d endPose = new Vector2d(0, 0);

    @Override
    public void runOpMode() {

        TankDrive drive = new TankDrive(hardwareMap, startPose);

        TrajectoryActionBuilder tab = drive.actionBuilder(startPose)
                .lineToX(20)
                .turn(Math.toRadians(90))
                .lineToY(20)
                .splineTo(endPose, Math.toRadians(90));

        Action trajectoryAction = tab.build();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(trajectoryAction);
        }
    }
}