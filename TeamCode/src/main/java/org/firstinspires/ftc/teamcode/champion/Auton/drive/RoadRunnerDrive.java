package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import static org.firstinspires.ftc.teamcode.champion.Auton.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.champion.Auton.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.champion.Auton.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.champion.Auton.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class RoadRunnerDrive {


    // Feedforward(guessing for motor power) + PID(correction)
    public static double TRANSLATIONAL_P = DriveConstants.TRANSLATIONAL_P;
    public static double TRANSLATIONAL_I = DriveConstants.TRANSLATIONAL_I;
    public static double TRANSLATIONAL_D = DriveConstants.TRANSLATIONAL_D;
    public static double HEADING_P = DriveConstants.HEADING_P;
    public static double HEADING_I = DriveConstants.HEADING_I;
    public static double HEADING_D = DriveConstants.HEADING_D;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(TRANSLATIONAL_P, TRANSLATIONAL_I, TRANSLATIONAL_D);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(HEADING_P, HEADING_I, HEADING_D);
    private TankPIDVAFollower follower;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    private final GoBildaPinpointDriver pinpoint;
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectorySequenceBuilder trajectorySequenceBuilder;

    private VoltageSensor batteryVoltageSensor;
    private PinpointLocalizer pinpointLocalizer;

    private double trackWidth = 12.0; // Distance between left and right drive wheels
    private double xOdoOffset = 6.0; // Distance from center of rotation to x odometry wheel

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

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

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public void initializePinpointPosition(LinearOpMode opMode, Pose2d pose2d) {
        while (!pinpointIsReady() && !opMode.isStopRequested()) {
            opMode.sleep(100);
        }

        setPoseEstimate(pose2d);
    }

    //build trajectories
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, getVelocityConstraint(), getAccelerationConstraint());
    }

    //follow trajectories
    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void update() {
        updateOdometry();
        Pose2d currentPose = getPoseEstimate();
        DriveSignal signal = follower.update(currentPose); //compares current pose to where it should be, outputs drive signal with velocity and acceleration
        // signal = null : trajectory finished
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
        double x_ = pose.getX() * 25.4;//inches to mm
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
                new AngularVelocityConstraint(MAX_ANG_VEL),//limit for turning
                new TankVelocityConstraint(MAX_VEL, DriveConstants.TrackWidth)
        ));
    }

    private ProfileAccelerationConstraint getAccelerationConstraint() {
        return new ProfileAccelerationConstraint(MAX_ACCEL);//limit for acceleration
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

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turn(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public boolean pinpointIsReady() {
        return pinpointLocalizer.isReady();
    }
}
