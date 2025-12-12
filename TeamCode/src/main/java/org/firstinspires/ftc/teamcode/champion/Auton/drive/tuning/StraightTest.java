package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

@Autonomous(name = "Straight Test", group = "Tuning")
public class StraightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d start = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, start, telemetry);

        telemetry.addLine("STRAIGHT LINE TEST");
        telemetry.addLine("Robot will drive 48 inches forward");
        telemetry.update();

        waitForStart();

        Action forward = drive.actionBuilder(start)
                .lineToX(48)
                .build();

        Actions.runBlocking(forward);

        // Get final pose
        Pose2d end = drive.localizer.getPose();
        double error = 48.0 - end.position.x;


        if (Math.abs(error) < 1.0) {
            telemetry.addLine("✓ EXCELLENT! Within 1 inch!");
        } else if (error > 1.0) {
            telemetry.addLine("⚠ UNDERSHOT");
            telemetry.addLine("→ Increase kV or wheelRadius");
        } else {
            telemetry.addLine("⚠ OVERSHOT");
            telemetry.addLine("→ Decrease kV or wheelRadius");
        }

        telemetry.update();
        sleep(30000);
    }
}