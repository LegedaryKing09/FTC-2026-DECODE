package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
@Config
@Autonomous(name = "Turn Tuning", group = "Tuning")
public class TurnTest extends LinearOpMode {
    AutoTankDrive tankDrive;
    // If jerk happens mostly when the robot is far away from target angle → lower max vel.
    // If jerk happens mostly near the target and looks like “hunting” → lower kP (and sometimes also max vel).
    public static double HEADING_CORRECTION_KP = 0.01;
    public static double HEADING_CORRECTION_MAX_VEL= 0.3;
    public static int HEADING_STABLE_SAMPLES = 3;
    public static double HEADING_TIMEOUT_MS = 300;
    public static double TURN_90 = 180.0;
    public static double TURN_45 = 90.0;
    public static double TURN_0 = 0.0;

    @Override
    public void runOpMode() {
        waitForStart();

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        turningMotion();

        // Wait for settling
        sleep(500);

    }

    // SEQUENCE : TURN TO 38, 90, 45
    private void turningMotion() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        Action turnLeft1 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(TURN_45))
                .build();
        Actions.runBlocking(turnLeft1);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turnLeft2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(TURN_90))
                .build();
        Actions.runBlocking(turnLeft2);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turnLeft3 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(TURN_45))
                .build();
        Actions.runBlocking(turnLeft3);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turnLeft4 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(TURN_0))
                .build();
        Actions.runBlocking(turnLeft4);


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

            sleep(10);
            attemptCount++;
        }

        tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(50);
    }

}
