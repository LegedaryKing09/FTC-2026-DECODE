package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import static org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.CB.HEADING_CORRECTION_KP;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.CB.HEADING_CORRECTION_MAX_VEL;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.CB.HEADING_STABLE_SAMPLES;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.CB.HEADING_TIMEOUT_MS;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.NewPathingForCB.SPLINE_Y;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton.CB;
import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;;

@Config
@Autonomous(name = "Spline Test For Auton", group = "Tuning")
public class SplineTestForAuton extends LinearOpMode {
    AutoTankDrive tankDrive;
    public static double X_DISTANCE = 25.0;
    public static double Y_DISTANCE = -10.0;
    public static double HEADING_CORRECTION_KP = 0.005;
    public static double HEADING_CORRECTION_MAX_VEL = 0.16;
    public static int HEADING_STABLE_SAMPLES = 3;
    public static double HEADING_TIMEOUT_MS = 280;

    @Override
    public void runOpMode() {
        waitForStart();

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        Spline();

        // Wait for settling
        sleep(500);

    }

    private void Spline() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        Action Initial_Forward = tankDrive.actionBuilder(currentPose)
                .lineToX(24)
                .build();
        Actions.runBlocking(Initial_Forward);
        HeadingCorrection(0,0.5);

        currentPose = tankDrive.pinpointLocalizer.getPose();

        Action Turn = tankDrive.actionBuilder(currentPose)
                .turnTo(-90)
                .build();
        Actions.runBlocking(Turn);
        HeadingCorrection(-90,0.5);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action Spline = tankDrive.actionBuilder(currentPose)
                .splineTo(new Vector2d(10, -10), 0)
                .build();
        Actions.runBlocking(Spline);
        HeadingCorrection(0,0.5);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action FORWARD = tankDrive.actionBuilder(currentPose)
                .lineToX(55)
                .build();
        Actions.runBlocking(FORWARD);

    }


    private void HeadingCorrection(double targetAngleDegrees, double toleranceDegrees) {
        final double kP = HEADING_CORRECTION_KP;
        final double maxAngularVel = HEADING_CORRECTION_MAX_VEL;
        final int stableSamplesRequired = HEADING_STABLE_SAMPLES;
        final double timeoutMs = HEADING_TIMEOUT_MS;

        final int maxAttempts = 15;
        int attemptCount = 0;
        int stableCount = 0;

        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opModeIsActive()
                && attemptCount < maxAttempts
                && timeout.milliseconds() < timeoutMs) {

            tankDrive.updatePoseEstimate();
            Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
            double currentAngleDeg = Math.toDegrees(currentPose.heading.toDouble());

            // normalize error
            double headingError = targetAngleDegrees - currentAngleDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            if (Math.abs(headingError) <= toleranceDegrees) {
                stableCount++;
                if (stableCount >= stableSamplesRequired) break;
            } else {
                stableCount = 0;
            }

            double angularVel = kP * headingError;
            angularVel = Math.max(-maxAngularVel, Math.min(maxAngularVel, angularVel));

            tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), angularVel));

            sleep(30);
            attemptCount++;
        }

        tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(50);
    }




}
