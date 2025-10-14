package org.firstinspires.ftc.teamcode.champion.controller;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
public class SixWheelDriveController {

    // Motor configuration names - STATIC like in version 2
    public static String LF_NAME = "lf";
    public static String RF_NAME = "rf";
    public static String LB_NAME = "lb";
    public static String RB_NAME = "rb";

    // Drive Motors (4 motors controlling 6 wheels via gears/belts)
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    // GoBilda Odometry Computer
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
        frontLeft = opMode.hardwareMap.get(DcMotor.class, LF_NAME);
        frontRight = opMode.hardwareMap.get(DcMotor.class, RF_NAME);
        backLeft = opMode.hardwareMap.get(DcMotor.class, LB_NAME);
        backRight = opMode.hardwareMap.get(DcMotor.class, RB_NAME);

        // Initialize GoBilda Odometry Computer
        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions - CORRECTED for tank drive
        // Left side needs to be consistent, right side needs to be consistent
        // For tank drive: both left motors same direction, both right motors same direction
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        setMotorsBrakeMode();

        // Configure the Pinpoint computer
        configurePinpoint();

        // Reset odometry
        resetOdometry();
    }

    // Configure the Pinpoint odometry computer
    private void configurePinpoint() {
        // Set encoder directions - adjust these based on your robot setup
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Set encoder resolution - using goBILDA swingarm pods as default
        // Change this to goBILDA_4_BAR_POD if using 4-bar pods, or use setEncoderResolution() for custom
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set pod offsets - these are the distances from the center of rotation to each odometry pod
        // X offset: positive = left of center, negative = right of center
        // Y offset: positive = forward of center, negative = behind center
        // Adjust these values based on your robot's physical configuration
        pinpoint.setOffsets(-3.59, 6, DistanceUnit.INCH); // Example values - ADJUST FOR YOUR ROBOT

        // Set yaw scalar if needed (usually not necessary as devices come pre-calibrated)
        // pinpoint.setYawScalar(1.0);
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

    // Update odometry calculations - THIS WAS THE MISSING PIECE
    public void updateOdometry() {
        // CRITICAL: Call update() first to refresh data from the Pinpoint device
        pinpoint.update();

        // Get the current position and heading from the Pinpoint driver
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeading = pose.getHeading(AngleUnit.RADIANS);
    }

    // Reset odometry encoders and position
    public void resetOdometry() {
        // Reset odometry computer's internal tracking and recalibrate IMU
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

        // Reset local position variables
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

    // Get position in different units
    public double getX(DistanceUnit unit) {
        return unit.fromMm(robotX * 25.4); // robotX is in inches, convert to mm first
    }

    public double getY(DistanceUnit unit) {
        return unit.fromMm(robotY * 25.4); // robotY is in inches, convert to mm first
    }

    public double getHeading(AngleUnit unit) {
        return unit.fromRadians(robotHeading);
    }

    // Setters for robot position (this will update the Pinpoint device)
    public void setPosition(double x, double y, double heading) {
        Pose2D newPose = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
        pinpoint.setPosition(newPose);
        robotX = DistanceUnit.MM.toInches(x);
        robotY = DistanceUnit.MM.toInches(y);
        robotHeading = heading;
    }

    public void setX(double x) {
        pinpoint.setPosX(x, DistanceUnit.MM);
        robotX = DistanceUnit.MM.toInches(x);
    }

    public void setY(double y) {
        pinpoint.setPosY(y, DistanceUnit.MM);
        robotY = DistanceUnit.MM.toInches(y);
    }

    public void setHeading(double heading) {
        pinpoint.setHeading(heading, AngleUnit.RADIANS);
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

    // Get velocities from the Pinpoint
    public double getVelocityX() {
        return pinpoint.getVelX(DistanceUnit.MM);
    }

    public double getVelocityY() {
        return pinpoint.getVelY(DistanceUnit.MM);
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
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

    // Additional utility methods from version 2
    public void setAllMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setLeftPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void setRightPower(double power) {
        frontRight.setPower(power);
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

    // Get pinpoint loop time and frequency for diagnostics
    public int getPinpointLoopTime() {
        return pinpoint.getLoopTime();
    }

    public double getPinpointFrequency() {
        return pinpoint.getFrequency();
    }

    // Speed mode methods
    public void setFastSpeed() {
        isFastSpeedMode = true;
    }

    public void setSlowSpeed() {
        isFastSpeedMode = false;
    }

    public boolean isFastSpeedMode() {
        return isFastSpeedMode;
    }

    // Test methods for debugging
    public void testAllMotorsDirectly(double power) {
        if (frontLeft != null) frontLeft.setPower(power);
        if (frontRight != null) frontRight.setPower(power);
        if (backLeft != null) backLeft.setPower(power);
        if (backRight != null) backRight.setPower(power);
    }

    // Get status information for telemetry
    public String getMotorStatus() {
        return "FL:" + (frontLeft != null ? "OK" : "NULL") +
                " FR:" + (frontRight != null ? "OK" : "NULL") +
                " BL:" + (backLeft != null ? "OK" : "NULL") +
                " BR:" + (backRight != null ? "OK" : "NULL");
    }

    @SuppressLint("DefaultLocale")
    public String getMotorPowers() {
        if (frontLeft == null || frontRight == null || backLeft == null || backRight == null) {
            return "Motors not initialized";
        }
        return String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());
    }

    // Get detailed pinpoint status for telemetry
    @SuppressLint("DefaultLocale")
    public String getPinpointTelemetry() {
        return String.format("Status: %s, Loop: %dμs (%.1fHz), X: %d, Y: %d",
                getPinpointStatus().toString(),
                getPinpointLoopTime(),
                getPinpointFrequency(),
                getXOdoPosition(),
                getYOdoPosition());
    }
}