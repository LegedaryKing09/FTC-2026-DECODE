package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class SixWheelDriveController {

    // Drive Motors (4 motors controlling 6 wheels via gears/belts)
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // GoBilda Odometry Computer
    private GoBildaPinpointDriver pinpoint = null;

    // Robot dimensions
    private double trackWidth = 12.0; // Distance between left and right drive wheels
    private double xOdoOffset = 6.0; // Distance from center of rotation to x odometry wheel

    // Robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Previous encoder readings (not used with the Pinpoint's internal odometry)
    private int prevXOdo = 0;
    private int prevYOdo = 0;
    private int prevLeftEncoder = 0;
    private int prevRightEncoder = 0;

    // Constructor
    public SixWheelDriveController() {
        // Empty constructor
    }

    // Initialize hardware
    public void init(HardwareMap hardwareMap) {
        // Initialize drive motors (4 motors controlling 6 wheels)
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");

        // Initialize GoBilda Odometry Computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        setMotorsBrakeMode();

        // Reset odometry
        resetOdometry();
    }

    // Tank drive control
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

    // Arcade drive control
    public void arcadeDrive(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        tankDrive(leftPower, rightPower);
    }

    // Stop all drive motors
    public void stopDrive() {
        tankDrive(0, 0);
    }

    // Update odometry calculations
    public void updateOdometry() {
        // Get the current position and heading directly from the Pinpoint driver
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.MM);
        robotY = pose.getY(DistanceUnit.MM);
        robotHeading = pose.getHeading(AngleUnit.DEGREES);

        // You can optionally convert the heading to degrees for display
        double robotHeadingDegrees = Math.toDegrees(robotHeading);

        // Note: The previous delta-based code is not necessary with the Pinpoint driver
    }

    // Reset odometry encoders and position
    public void resetOdometry() {
        // Reset odometry computer's internal tracking
        pinpoint.resetPosAndIMU();

        // Reset drive motor encoders (unnecessary for odometry, but good practice for other uses)
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset tracking variables
        prevXOdo = 0;
        prevYOdo = 0;
        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
    }

    // Set motors to brake mode
    private void setMotorsBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Getters for robot position
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

    // Setters for robot position
    public void setPosition(double x, double y, double heading) {
        robotX = x;
        robotY = y;
        robotHeading = heading;
    }

    public void setX(double x) {
        robotX = x;
    }

    public void setY(double y) {
        robotY = y;
    }

    public void setHeading(double heading) {
        robotHeading = heading;
    }

    // Getters and setters for robot dimensions
    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double getXOdoOffset() {
        return xOdoOffset;
    }

    public void setXOdoOffset(double xOdoOffset) {
        this.xOdoOffset = xOdoOffset;
    }

    // Get raw odometry encoder values (for debugging purposes)
    public int getXOdoPosition() {
        return pinpoint.getEncoderX();
    }

    public int getYOdoPosition() {
        return pinpoint.getEncoderY();
    }

    // Individual motor control
    public void setFrontLeftPower(double power) {
        frontLeft.setPower(power);
    }

    public void setFrontRightPower(double power) {
        frontRight.setPower(power);
    }

    public void setBackLeftPower(double power) {
        backLeft.setPower(power);
    }

    public void setBackRightPower(double power) {
        backRight.setPower(power);
    }

    // Get pinpoint odometry computer reference for advanced usage
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    // Set pinpoint offsets (distance from center of rotation to odometry pods)
    public void setPinpointOffsets(double xOffset, double yOffset) {
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
    }

    // Get pinpoint status
    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }
}