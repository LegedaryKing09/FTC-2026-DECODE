package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

@Autonomous(name = "Turn Test", group = "Tuning")
public class TurnTest extends LinearOpMode {
    public double turnAngle = 90;
    @Override
    public void runOpMode() {
        Pose2d start = new Pose2d(0, 0, 0);
        AutoTankDrive drive = new AutoTankDrive(hardwareMap, start, telemetry);

        telemetry.addLine("TRACK WIDTH TUNING");
        telemetry.addLine("Robot will turn 360° (4×90°)");
        telemetry.addData("Current Track Width", "%.2f inches",
                AutoTankDrive.PARAMS.physicalTrackWidthInches);
        telemetry.update();

        waitForStart();

        // Turn 360° (should end at 0°)
        Action turn360 = drive.actionBuilder(start)
                .turn(Math.toRadians(turnAngle))
                .turn(Math.toRadians(turnAngle))
                .turn(Math.toRadians(turnAngle))
                .turn(Math.toRadians(turnAngle))
                .build();

        Actions.runBlocking(turn360);

        Pose2d end = drive.localizer.getPose();
        double headingDeg = Math.toDegrees(end.heading.toDouble());

        // Normalize to -180 to 180
        while (headingDeg > 180) headingDeg -= 360;
        while (headingDeg < -180) headingDeg += 360;

        telemetry.clear();
        telemetry.addLine("==================================");
        telemetry.addLine("TURN TEST COMPLETE");
        telemetry.addLine("==================================");
        telemetry.addData("Expected Heading", "0.0°");
        telemetry.addData("Actual Heading", "%.2f°", headingDeg);
        telemetry.addData("Error", "%.2f°", headingDeg);
        telemetry.addLine();

        if (Math.abs(headingDeg) < 2.0) {
            telemetry.addLine("✓ EXCELLENT! Track width is correct!");
        } else if (headingDeg > 2.0) {
            telemetry.addLine("⚠ OVERTURNED (turned too much)");
            telemetry.addLine("→ Track width is TOO SMALL");
            double newTrackWidth = AutoTankDrive.PARAMS.physicalTrackWidthInches *
                    (360.0 / (360.0 + headingDeg));
            telemetry.addData("Current", "%.2f inches",
                    AutoTankDrive.PARAMS.physicalTrackWidthInches);
            telemetry.addData("Try", "%.2f inches", newTrackWidth);
        } else {
            telemetry.addLine("⚠ UNDERTURNED (didn't turn enough)");
            telemetry.addLine("→ Track width is TOO LARGE");
            double newTrackWidth = AutoTankDrive.PARAMS.physicalTrackWidthInches *
                    (360.0 / (360.0 + headingDeg));
            telemetry.addData("Current", "%.2f inches",
                    AutoTankDrive.PARAMS.physicalTrackWidthInches);
            telemetry.addData("Try", "%.2f inches", newTrackWidth);
        }

        telemetry.update();
        sleep(30000);
    }
}