package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.TankDrive;

@Config
@Autonomous
public class RoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        TankDrive.PARAMS.inPerTick = 0.02;
        TankDrive.PARAMS.trackWidthTicks = 600;
        TankDrive.PARAMS.kS = 0.1;
        TankDrive.PARAMS.kV = 0.01;
        TankDrive.PARAMS.kA = 0.002;
        TankDrive.PARAMS.maxWheelVel = 20;
        TankDrive.PARAMS.maxProfileAccel = 15;
        TankDrive.PARAMS.minProfileAccel = -15;
        TankDrive.PARAMS.maxAngVel = Math.toRadians(60);
        TankDrive.PARAMS.maxAngAccel = Math.toRadians(60);

        // starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        TankDrive drive = new TankDrive(hardwareMap, startPose);

//        TrajectoryActionBuilder forwardsTurnAndReturn = drive.actionBuilder(startPose)
//                .lineToX(20)
//                .turn(Math.toRadians(90))
//                .lineToY(20)
//                .splineTo(endPose, Math.toRadians(90));
//
//        Action build = forwardsTurnAndReturn.build();

        // Create trajectory actions using actionBuilder
        Action moveForwardAndTurn = drive.actionBuilder(startPose)
                .lineToX(24)  // Move forward 24 inches
                .turn(Math.toRadians(90))  // Turn left 90 degrees
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

}}