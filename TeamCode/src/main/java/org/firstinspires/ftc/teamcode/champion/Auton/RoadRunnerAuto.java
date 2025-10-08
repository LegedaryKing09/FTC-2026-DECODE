package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.TankDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {

    public static double ForwardInch = 24.0;
    public static double BackwardInch = -8.0;
    public static double TurnAngle = 90.0;
    public static double SecondForward = 4.0;
    public static double SecondTurn = 180.0;


    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        TankDrive drive = new TankDrive(hardwareMap, startPose);

        Action moveForwardAndTurn = drive.actionBuilder(startPose)
            .lineToX(ForwardInch)  // Move forward 24 inches
          //  .turn(Math.toRadians(TurnAngle))  // Turn left 90 degree
                .turn(Math.toRadians(SecondTurn)) //Turn left 90 degrees
                 //.lineToX(0)
                .build();

        waitForStart();

        Actions.runBlocking(moveForwardAndTurn);

    }
}