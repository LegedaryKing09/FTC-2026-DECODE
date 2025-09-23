package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.TankDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {

    public static double ForwardInch = 24.0;
    public static double TurnAngle = 90.0;

    @Override
    public void runOpMode() {

        // starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        TankDrive drive = new TankDrive(hardwareMap, startPose);

        Action moveForwardAndTurn = drive.actionBuilder(startPose)
                .lineToX(ForwardInch)  // Move forward 24 inches
                .turn(Math.toRadians(TurnAngle))  // Turn left 90 degrees
                .build();

        waitForStart();

        if (isStopRequested()) return;

        try {
            // action sequence
            Actions.runBlocking(moveForwardAndTurn);

        } catch (Exception e) {
            // Emergency stop - set all motor powers to 0
            stopAllMotors(drive);
        }

        sleep(1000);
    }

    private void stopAllMotors(TankDrive drive) {
            // Stop left motors
            if (drive.leftMotors != null) {
                for (int i = 0; i < drive.leftMotors.size(); i++) {
                    drive.leftMotors.get(i).setPower(0);
                }
            }

            // Stop right motors
            if (drive.rightMotors != null) {
                for (int i = 0; i < drive.rightMotors.size(); i++) {
                    drive.rightMotors.get(i).setPower(0);
                }
            }
    }
}