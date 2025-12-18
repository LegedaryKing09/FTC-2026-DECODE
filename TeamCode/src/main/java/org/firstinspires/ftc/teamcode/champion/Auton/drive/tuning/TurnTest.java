package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;
@Config
@Autonomous(name = "Turn Tuning", group = "Tuning")
public class TurnTest extends LinearOpMode {
    public static double TURN_ANGLE = 90;  // Change via FTC Dashboard

    @Override
    public void runOpMode() {
        Pose2d start = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, start);

        waitForStart();

        while (opModeIsActive()) {
            // Reset position
//            drive.pinpointLocalizer.setPose(start);

            double turnRadians = Math.toRadians(TURN_ANGLE);
            Action turnTest = drive.actionBuilder(start)
                    .turn(turnRadians)
                    .build();

            Actions.runBlocking(turnTest);

            // Show results
            double finalHeading = Math.toDegrees(drive.pinpointLocalizer.getPose().heading.toDouble());
            double error = TURN_ANGLE - finalHeading;

            telemetry.addData("Target Heading", "%.1f°", TURN_ANGLE);
            telemetry.addData("Final Heading", "%.1f°", finalHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("", "");
            telemetry.addData("turnKS", "%.3f", AutoTankDrive.PARAMS.kS);
            telemetry.addData("turnKV", "%.3f", AutoTankDrive.PARAMS.kV);
            telemetry.addData("turnKA", "%.4f", AutoTankDrive.PARAMS.kA);
            telemetry.addData("", "");
            telemetry.addData("Status", "Press START to run again");
            telemetry.update();

            // Wait for next run
            while (opModeIsActive() && !gamepad1.start) {
                sleep(20);
            }
            while (opModeIsActive() && gamepad1.start) {
                sleep(20);
            }
        }
    }
}