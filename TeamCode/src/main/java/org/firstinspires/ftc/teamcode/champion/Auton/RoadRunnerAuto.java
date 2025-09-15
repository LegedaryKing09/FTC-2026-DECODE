package org.firstinspires.ftc.teamcode.champion.Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;


@Config
public  class DriveConstants {
    public double TrackWidth = 12.0;
    // Movement limits
    public double MAX_VEL = 30; // inches per second
    public double MAX_ACCEL = 30; // inches per second squared
    public double MAX_ANG_VEL = Math.toRadians(120); // radians per second
    public double MAX_ANG_ACCEL = Math.toRadians(120); // radians per second squared

    // with pinpoint, we don't need motor encoder constants, it handles all the odometry calculations
}
    public class RoadRunnerDrive {
        private SixWheelDriveController driveController;
        private TankPIDVAFollower follower;

        public RoadRunnerDrive(HardwareMap hardwareMap) {
            SixWheelDriveController driveController = new SixWheelDriveController();
            driveController.init(hardwareMap);

            follower = new TankPIDVAFollower(
                    new PIDCoefficients(8.0, 0.0, 0.0); //forward PID, too low = slow, too high = oscillation
            new PIDCoefficients(5.0, 0.0, 0.0); //backward PID, too low = drifting, too high = wobbling
            new PIDCoefficients(5.0, 0.0, 0.0); //heading PID, too low = poor turns, too high = overshoot
            )
        }

        public TrajectoryBuilder trajectoryBuilder(Pose2D startPose) {
            return new TrajectoryBuilder(startPose);
        }

        public TrajectoryBuilder trajectoryBuilder(Pose2D startPose, double startHeading) {
            return new TrajectoryBuilder(startPose, startHeading)
        }

        public void followTrajectory(Trajectory trajectory) {
            follower.followTrajectory(trajectory);
        }

        public void update() {
            driveController.updateOdometry();
            Pose2d currentPose = getPoseEstimate();
            DriveSignal signal = follower.update(currentPose); //holding velocity and acceleration commands
            if (signal !null)
        }


        //current robot pose from Pinpoint
        public Pose2d getPoseEstimate() {
            driveController.updateOdometry();
            double x = driveController.getX() / 25.4; //conversion from mm to inches
            double y = driveController.getY() / 25.4;
            double heading = Math.toRadians(driveController.getHeadingDegrees());
            return new Pose2d(x, y, heading);
        }

        //setting starting pose
        public void setPoseEstimate(Pose2D pose) {
            double x_ = pose.getX(DistanceUnit.MM) * 25.4;
            double y_ = pose.getY(DistanceUnit.MM) * 25.4;
            double headingDegree = Math.toDegrees(pose.getHeading(AngleUnit.RADIANS));
            driveController.setPosition(x_, y_, Math.toRadians(headingDegree));
        }


        //checking if trajectory is still running
        public boolean isBusy() {
            return follower.isFollowing();
        }

        //Manual drive control, from SixWheelDriveController
        public void tankDrive(double leftPower, double rightPower) {
            driveController.tankDrive(leftPower, rightPower);
        }

        public void arcadeDrive(double drive, double turn) {
            driveController.arcadeDrive(drive, turn);
        }

        public void stop() {
            driveController.stopDrive();
        }

        public SixWheelDriveController getDriveController() {
            return driveController;
        }
    }

    @Config
    @Autonomous
    public class RoadRunnerAuto extends LinearOpMode{

        @Override
        public void runOpMode(){
            RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap);
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Pinpoint Status", drive.getDriveController().getPinpointStatus());
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            // BASIC MOVEMENTS EXAMPLE:

            // 1. Move forward 24 inches
            Trajectory moveForward = drive.trajectoryBuilder(startPose)
                    .forward(24)
                    .build();

            drive.followTrajectory(moveForward);
            while (drive.isBusy() && !isStopRequested()) {
                drive.update();

                Pose2d currentPose = drive.getPoseEstimate();
                telemetry.addData("Status", "Moving Forward");
                telemetry.addData("x", currentPose.getX());
                telemetry.addData("y", currentPose.getY());
                telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
                telemetry.update();
            }

            // 2. Turn left 90 degrees
            Trajectory turnLeft = drive.trajectoryBuilder(moveForward.end())
                    .turn(Math.toRadians(90))
                    .build();

            drive.followTrajectory(turnLeft);
            while (drive.isBusy() && !isStopRequested()) {
                drive.update();
                telemetry.addData("Status", "Turning Left");
                telemetry.update();
            }

            // 3. Move backward 12 inches
            Trajectory moveBack = drive.trajectoryBuilder(turnLeft.end())
                    .back(12)
                    .build();

            drive.followTrajectory(moveBack);
            while (drive.isBusy() && !isStopRequested()) {
                drive.update();
                telemetry.addData("Status", "Moving Back");
                telemetry.update();
            }

            telemetry.addData("Status", "Complete!");
            telemetry.update();

            // Stop motors
            drive.stop();
        }

        }

    }


}