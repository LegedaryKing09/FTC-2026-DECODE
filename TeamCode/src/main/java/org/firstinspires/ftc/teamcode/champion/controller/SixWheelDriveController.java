package org.firstinspires.ftc.teamcode.champion.controller;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class SixWheelDriveController {

    // Motor configuration names - STATIC like in version 2
    public static String LF_NAME = "lf";
    public static String RF_NAME = "rf";
    public static String LB_NAME = "lb";
    public static String RB_NAME = "rb";

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private final GoBildaPinpointDriver pinpoint;

    // Robot dimensions
    private double trackWidth = 12.0; // Distance between left and right drive wheels
    private double xOdoOffset = 6.0; // Distance from center of rotation to x odometry wheel

    // Robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Speed mode settings
    public static double FAST_SPEED_MULTIPLIER = 6;
    public static double FAST_TURN_MULTIPLIER = 7;
    public static double SLOW_SPEED_MULTIPLIER = 0.8;
    public static double SLOW_TURN_MULTIPLIER = 3.5;

    private boolean isFastSpeedMode = false;

    // Constructor
    public SixWheelDriveController(OpMode opMode) {
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, LF_NAME);
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, RF_NAME);
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, LB_NAME);
        backRight = opMode.hardwareMap.get(DcMotorEx.class, RB_NAME);

        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, LF_NAME);
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, RF_NAME);
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, LB_NAME);
        backRight = opMode.hardwareMap.get(DcMotorEx.class, RB_NAME);

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setMotorsBrakeMode();
        configurePinpoint();
        initializeVelocityControl();
        resetOdometry();
    }

    private void configurePinpoint() {
        // Set encoder directions based on config
        GoBildaPinpointDriver.EncoderDirection xDir = OdometryParams.X_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaPinpointDriver.EncoderDirection yDir = OdometryParams.Y_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        pinpoint.setEncoderDirections(xDir, yDir);

        // Set correct pod type
        if (OdometryParams.USE_4_BAR_PODS) {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        } else {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        }

        // Set offsets in MM (consistent units)
        pinpoint.setOffsets(OdometryParams.X_OFFSET_MM, OdometryParams.Y_OFFSET_MM, DistanceUnit.MM);

        // Set yaw scalar for heading calibration
        pinpoint.setYawScalar(OdometryParams.YAW_SCALAR);
    }

    private void initializeVelocityControl() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPIDF(frontLeft);
        setMotorPIDF(frontRight);
        setMotorPIDF(backLeft);
        setMotorPIDF(backRight);
    }

    private void setMotorPIDF(DcMotorEx motor) {
        motor.setVelocityPIDFCoefficients(
                VelocityParams.VELOCITY_P,
                VelocityParams.VELOCITY_I,
                VelocityParams.VELOCITY_D,
                VelocityParams.VELOCITY_F
        );
    }

    public void tankDrive(double leftPower, double rightPower) {
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    public void arcadeDrive(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        tankDrive(leftPower, rightPower);
    }

    public void stopDrive() {
        tankDrive(0, 0);
    }

    public void updateOdometry() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        // FIXED: Use consistent units throughout - stick with INCHES
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeading = pose.getHeading(AngleUnit.RADIANS);
    }

    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
        initializeVelocityControl();
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
    }

    private void setMotorsBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Getters
    public boolean isFastSpeedMode() {
        return isFastSpeedMode;
    }

    public void setFastSpeed() {
        isFastSpeedMode = true;
    }

    public void setSlowSpeed() {
        isFastSpeedMode = false;
    }

    public void toggleSpeedMode() {
        isFastSpeedMode = !isFastSpeedMode;
    }

    // Odometry getters - all in INCHES for consistency
    public double getX() {
        return robotX;
    }

    public double getY() {
        return robotY;
    }

    public double getHeading() {
        return robotHeading;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(robotHeading);
    }

    // Raw encoder positions for debugging
    public int getXOdoPosition() {
        return pinpoint.getEncoderX();
    }

    public int getYOdoPosition() {
        return pinpoint.getEncoderY();
    }

    // Velocities
    public double getVelocityX() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getVelocityY() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void setPosition(double x, double y, double heading) {
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
        pinpoint.setPosition(newPose);
        robotX = x;
        robotY = y;
        robotHeading = heading;
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }

    public int getPinpointLoopTime() {
        return pinpoint.getLoopTime();
    }

    public double getPinpointFrequency() {
        return pinpoint.getFrequency();
    }

    // Velocity control methods
    public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
        leftVelocity = Range.clip(leftVelocity,
                -VelocityParams.MAX_TICKS_PER_SEC, VelocityParams.MAX_TICKS_PER_SEC);
        rightVelocity = Range.clip(rightVelocity,
                -VelocityParams.MAX_TICKS_PER_SEC, VelocityParams.MAX_TICKS_PER_SEC);

        frontLeft.setVelocity(leftVelocity);
        backLeft.setVelocity(leftVelocity);
        frontRight.setVelocity(rightVelocity);
        backRight.setVelocity(rightVelocity);
    }

    public void tankDriveVelocityNormalized(double leftPower, double rightPower) {
        double leftVel = leftPower * VelocityParams.MAX_TICKS_PER_SEC;
        double rightVel = rightPower * VelocityParams.MAX_TICKS_PER_SEC;
        tankDriveVelocity(leftVel, rightVel);
    }

    public double getLeftVelocity() {
        return (frontLeft.getVelocity() + backLeft.getVelocity()) / 2.0;
    }

    public double getRightVelocity() {
        return (frontRight.getVelocity() + backRight.getVelocity()) / 2.0;
    }

    // Motor status telemetry
    @SuppressLint("DefaultLocale")
    public void getMotorStatus() {
        opMode.telemetry.addData("Front Left Power", "%.2f", frontLeft.getPower());
        opMode.telemetry.addData("Front Right Power", "%.2f", frontRight.getPower());
        opMode.telemetry.addData("Back Left Power", "%.2f", backLeft.getPower());
        opMode.telemetry.addData("Back Right Power", "%.2f", backRight.getPower());
    }

    @SuppressLint("DefaultLocale")
    public String getMotorPowers() {
        return String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());
    }
}