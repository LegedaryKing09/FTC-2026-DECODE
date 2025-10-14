package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto - Odometry Check", group="Diagnostics")
public class Auto_OdoCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, startPose);

        // Initialize pinpoint
        drive.initializePinpoint(this, startPose);

        telemetry.addLine("====================================");
        telemetry.addLine("ODOMETRY CHECK MODE");
        telemetry.addLine("====================================");
        telemetry.addLine("Ready! Press START");
        telemetry.addLine("");
        telemetry.addLine("After starting:");
        telemetry.addLine("1. Push robot forward 24 inches");
        telemetry.addLine("2. Check if X shows ~24");
        telemetry.addLine("3. Turn robot 180 degrees");
        telemetry.addLine("4. Check if Heading shows ~180°");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Status", drive.pinpoint.getDeviceStatus());
            telemetry.addData("X (inches)", "%.2f", drive.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", "%.2f", drive.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addLine("");
            telemetry.addLine("Push robot to test odometry!");
            telemetry.update();

            sleep(50);
        }
    }
}