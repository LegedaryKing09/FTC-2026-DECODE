package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.champion.Auton.DriveConstants;

import java.util.Arrays;

@Config
public class RoadRunnerDrive {
    private TankPIDVAFollower follower;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    private final GoBildaPinpointDriver pinpoint;

    private double trackWidth = 12.0; // Distance between left and right drive wheels
    private double xOdoOffset = 6.0; // Distance from center of rotation to x odometry wheel

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    public RoadRunnerDrive(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, SixWheelDriveController.LF_NAME);
        frontRight = hardwareMap.get(DcMotor.class, SixWheelDriveController.RF_NAME);
        backLeft = hardwareMap.get(DcMotor.class, SixWheelDriveController.LB_NAME);
        backRight = hardwareMap.get(DcMotor.class, SixWheelDriveController.RB_NAME);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        updateOdometry();
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
            tankDrive(leftPower, rightPower);
        } else {
            stopDrive();
        }
    }

    //current robot pose from Pinpoint
    public Pose2d getPoseEstimate() {
        updateOdometry();
        double x = getX() / 25.4; //conversion from mm to inches
        double y = getY() / 25.4;
        double heading = Math.toRadians(getHeadingDegrees());
        return new Pose2d(x, y, heading);
    }

    //setting starting pose
    public void setPoseEstimate(Pose2d pose) {
        double x_ = pose.getX() * 25.4;
        double y_ = pose.getY() * 25.4;
        double headingDegree = pose.getHeading();
        setPosition(x_, y_, Math.toRadians(headingDegree));
    }


    //checking if trajectory is still running
    public boolean isBusy() {
        return follower.isFollowing();
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

    public int getXOdoPosition() {
        return pinpoint.getEncoderX();
    }

    public int getYOdoPosition() {
        return pinpoint.getEncoderY();
    }

    public void setAllMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void arcadeDrive(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        tankDrive(leftPower, rightPower);
    }

    public void tankDrive(double leftPower, double rightPower) {
        // Normalize powers if they exceed 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Set left side motors (controls front, middle, and back wheels on left)
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);

        // Set right side motors (controls front, middle, and back wheels on right)
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    public void updateOdometry() {
        // Get the current position and heading directly from the Pinpoint driver
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.MM);
        robotY = pose.getY(DistanceUnit.MM);
        robotHeading = pose.getHeading(AngleUnit.DEGREES);
        // Note: The previous delta-based code is not necessary with the Pinpoint driver
    }

    public void stopDrive() {
        tankDrive(0, 0);
    }

    public double getX() {
        return robotX;
    }

    public double getY() {
        return robotY;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(robotHeading);
    }

    // Setters for robot position
    public void setPosition(double x, double y, double heading) {
        robotX = x;
        robotY = y;
        robotHeading = heading;
    }
}
