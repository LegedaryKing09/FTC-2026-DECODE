package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

@Autonomous(name = "Heading Direction Test", group = "Tuning")
public class HeadingDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d start = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, start);

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            double headingDeg = Math.toDegrees(drive.pinpointLocalizer.getPose().heading.toDouble());

            telemetry.addData("Current Heading (deg)", "%.2f", headingDeg);
            telemetry.addData("", "");
            telemetry.addData("INSTRUCTIONS:", "");
            telemetry.addData("1.", "Manually rotate robot LEFT (counterclockwise from above)");
            telemetry.addData("2.", "Heading should INCREASE (0° → 90° → 180°)");
            telemetry.addData("", "");
            telemetry.addData("If heading DECREASES", "your heading is INVERTED");
            telemetry.update();
        }
    }
}