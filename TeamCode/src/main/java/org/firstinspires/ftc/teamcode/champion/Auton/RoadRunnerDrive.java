package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import java.util.Arrays;

public class RoadRunnerDrive {
    private SixWheelDriveController driveController;
    private TankPIDVAFollower follower;
    public RoadRunnerDrive(HardwareMap hardwareMap) {
        SixWheelDriveController driveController = new SixWheelDriveController();
        driveController.init(hardwareMap);
        follower = new TankPIDVAFollower(
                new PIDCoefficients(5.0, 0.0, 0.0),   // Forward/back
                new PIDCoefficients(0.0, 0.0, 0.0)   // Drift (disable if no strafing)
        );
    }

    //build trajectories
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose,getVelocityConstraint(),getAccelerationConstraint());
    }

    //follow trajectories
    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void update() {
        driveController.updateOdometry();
        Pose2d currentPose = getPoseEstimate();
        DriveSignal signal = follower.update(currentPose); //holding velocity and acceleration commands
        if (signal != null) {
            // Convert DriveSignal to tank drive powers
            double forward = signal.getVel().getX();     // Forward/backward velocity
            double turn = signal.getVel().getHeading();  // Turning velocity

            // Tank drive: left = forward - turn, right = forward + turn
            double leftPower = forward - turn;
            double rightPower = forward + turn;
            // Control your robot
            driveController.tankDrive(leftPower, rightPower);
        } else {
            driveController.stopDrive();
        }
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
    public void setPoseEstimate(Pose2d pose) {
        double x_ = pose.getX() * 25.4;
        double y_ = pose.getY() * 25.4;
        double headingDegree = pose.getHeading();
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

    private MinVelocityConstraint getVelocityConstraint() {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new TankVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TrackWidth)
        ));
    }
    private ProfileAccelerationConstraint getAccelerationConstraint() {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }
}
