package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

@Config  // ← This enables FTC Dashboard configuration
@Autonomous(name = "Straight Test", group = "Tuning")
public class StraightTest extends LinearOpMode {

    // Static variables are exposed to FTC Dashboard
    public static double forwardDistance = 24;
    public static double endingDistance = 0;

    @Override
    public void runOpMode() {
        Pose2d start = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, start);

        waitForStart();

        Action forward = drive.actionBuilder(start)
                .lineToX(forwardDistance)
                .lineToX(endingDistance)
                .build();

        Actions.runBlocking(forward);




        telemetry.update();
        sleep(30000);
    }
}