package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(AutoTankDrive.class)) {
            waitForStart();

            while (opModeIsActive()) {
                AutoTankDrive drive = new AutoTankDrive(hardwareMap, beginPose);
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(24, 24), Math.PI / 2)
                                .splineTo(new Vector2d(0, 48), Math.PI)
                                .build());
                while (opModeIsActive() && !gamepad1.x)
                    sleep(1);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
