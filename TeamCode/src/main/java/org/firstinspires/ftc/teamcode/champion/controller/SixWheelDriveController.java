package org.firstinspires.ftc.teamcode.champion.controller;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Config
public class SixWheelDriveController {

    // Motor configuration names
    public static String LF_NAME = "lf";
    public static String RF_NAME = "rf";
    public static String LB_NAME = "lb";
    public static String RB_NAME = "rb";

    private final DcMotorEx lf;
    private final DcMotorEx rf;
    private final DcMotorEx lb;
    private final DcMotorEx rb;

    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode linearOpMode;
    private final OpMode iterativeOpMode;

    // Robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Previous position for smoothing and drift correction
    private double lastRobotX = 0.0;
    private double lastRobotY = 0.0;
    private double lastRobotHeading = 0.0;
    private long lastOdometryUpdate = 0;

    // IMU calibration and drift tracking
    private double imuDriftRate = 0.0;
    private long lastImuCalibration = 0;

    // Speed mode settings - ADJUSTED FOR BETTER CONTROL
    // Reduced forward/backward speeds for better control
    public static double FAST_SPEED_MULTIPLIER = 0.8;  // Reduced from 0.3
    public static double FAST_TURN_MULTIPLIER = 0.4;    // Significantly reduced from 1.5
    public static double SLOW_SPEED_MULTIPLIER = 0.2;
    public static double SLOW_TURN_MULTIPLIER = 0.2;    // Significantly reduced from 0.8

    // Acceleration ramping to reduce inertia issues
    public static double ACCELERATION_RATE = 0.2;  // How quickly to ramp up/down power
    private double currentDrivePower = 0.0;
    private double currentTurnPower = 0.0;

    private boolean isFastSpeedMode = false;

    // Control mode
    public enum DriveMode {
        POWER,
        VELOCITY
    }
    private DriveMode currentDriveMode = DriveMode.VELOCITY;

    @Config
    public static class VelocityParams {
        public static double TICKS_PER_REV = 751.8;
        public static double MAX_RPM = 312.0;
        public static double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        // TUNABLE PID - Reduced P and D for smoother response
        public static double VELOCITY_P = 26;  // Reduced from 29
        public static double VELOCITY_I = 0.0;
        public static double VELOCITY_D = 0.1;  // Reduced from 0.2
        public static double VELOCITY_F = 12.0;
        public static double TRACK_WIDTH_INCHES = 11.5;
        public static double WHEEL_DIAMETER_INCHES = 2.83;
    }

    @Config
    public static class OdometryParams {
        public static boolean USE_4_BAR_PODS = true;
        public static double X_OFFSET_MM = -84.0;
        public static double Y_OFFSET_MM = -168.0;
        public static double YAW_SCALAR = 1.0;
        public static boolean X_ENCODER_REVERSED = false;
        public static boolean Y_ENCODER_REVERSED = false;

        public static double POSITION_SMOOTHING_FACTOR = 0.9;
    }

    // Constructor for LinearOpMode
    public SixWheelDriveController(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        this.iterativeOpMode = null;

        lf = opMode.hardwareMap.get(DcMotorEx.class, LF_NAME);
        rf = opMode.hardwareMap.get(DcMotorEx.class, RF_NAME);
        lb = opMode.hardwareMap.get(DcMotorEx.class, LB_NAME);
        rb = opMode.hardwareMap.get(DcMotorEx.class, RB_NAME);

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorsBrakeMode();
        configurePinpoint();
        initializeVelocityControl();
        resetOdometry();
    }

    // Constructor for OpMode (iterative)
    public SixWheelDriveController(OpMode opMode) {
        this.linearOpMode = null;
        this.iterativeOpMode = opMode;

        lf = opMode.hardwareMap.get(DcMotorEx.class, LF_NAME);
        rf = opMode.hardwareMap.get(DcMotorEx.class, RF_NAME);
        lb = opMode.hardwareMap.get(DcMotorEx.class, LB_NAME);
        rb = opMode.hardwareMap.get(DcMotorEx.class, RB_NAME);

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorsBrakeMode();
        configurePinpoint();
        initializeVelocityControl();
        resetOdometry();
    }

    private void configurePinpoint() {
        GoBildaPinpointDriver.EncoderDirection xDir = OdometryParams.X_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaPinpointDriver.EncoderDirection yDir = OdometryParams.Y_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        pinpoint.setEncoderDirections(xDir, yDir);

        if (OdometryParams.USE_4_BAR_PODS) {
            pinpoint.setEncoderResolution(19.9847352f, DistanceUnit.MM);
        } else {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        }

        pinpoint.setOffsets(OdometryParams.X_OFFSET_MM, OdometryParams.Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setYawScalar(OdometryParams.YAW_SCALAR);
    }

    private void initializeVelocityControl() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPIDF(lf);
        setMotorPIDF(rf);
        setMotorPIDF(lb);
        setMotorPIDF(rb);
    }

    private void setMotorPIDF(DcMotorEx motor) {
        motor.setVelocityPIDFCoefficients(
                VelocityParams.VELOCITY_P,
                VelocityParams.VELOCITY_I,
                VelocityParams.VELOCITY_D,
                VelocityParams.VELOCITY_F
        );
    }

    // === DRIVE CONTROL METHODS ===

    public void setDriveMode(DriveMode mode) {
        this.currentDriveMode = mode;
    }

    public DriveMode getCurrentDriveMode() {
        return currentDriveMode;
    }

    public void arcadeDrive(double drive, double turn) {
        // Apply acceleration ramping to smooth out sudden changes

        // Smooth the drive power changes
        if (Math.abs(drive - currentDrivePower) > ACCELERATION_RATE) {
            if (drive > currentDrivePower) {
                currentDrivePower += ACCELERATION_RATE;
            } else {
                currentDrivePower -= ACCELERATION_RATE;
            }
        } else {
            currentDrivePower = drive;
        }

        // Smooth the turn power changes
        if (Math.abs(turn - currentTurnPower) > ACCELERATION_RATE) {
            if (turn > currentTurnPower) {
                currentTurnPower += ACCELERATION_RATE;
            } else {
                currentTurnPower -= ACCELERATION_RATE;
            }
        } else {
            currentTurnPower = turn;
        }

        if (currentDriveMode == DriveMode.VELOCITY) {
            arcadeDriveVelocity(currentDrivePower, currentTurnPower);
        } else {
            arcadeDrivePower(currentDrivePower, currentTurnPower);
        }
    }

    private void arcadeDrivePower(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        tankDrive(leftPower, rightPower);
    }

    private void arcadeDriveVelocity(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        tankDriveVelocityNormalized(leftPower, rightPower);
    }

    public void tankDrive(double leftPower, double rightPower) {
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }
        lf.setPower(leftPower);
        lb.setPower(leftPower);
        rf.setPower(rightPower);
        rb.setPower(rightPower);
    }

    public void stopDrive() {
        // Reset current power values for smooth stop
        currentDrivePower = 0;
        currentTurnPower = 0;

        if (currentDriveMode == DriveMode.VELOCITY) {
            tankDriveVelocity(0, 0);
        } else {
            tankDrive(0, 0);
        }
    }

    // === VELOCITY CONTROL METHODS ===

    public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
        leftVelocity = Range.clip(leftVelocity,
                -VelocityParams.MAX_TICKS_PER_SEC, VelocityParams.MAX_TICKS_PER_SEC);
        rightVelocity = Range.clip(rightVelocity,
                -VelocityParams.MAX_TICKS_PER_SEC, VelocityParams.MAX_TICKS_PER_SEC);

        lf.setVelocity(leftVelocity);
        lb.setVelocity(leftVelocity);
        rf.setVelocity(rightVelocity);
        rb.setVelocity(rightVelocity);
    }

    public void tankDriveVelocityNormalized(double leftPower, double rightPower) {
        // FIXED: Removed velocity scaling - use full range as specified by PID
        double leftVel = leftPower * VelocityParams.MAX_TICKS_PER_SEC;
        double rightVel = rightPower * VelocityParams.MAX_TICKS_PER_SEC;
        tankDriveVelocity(leftVel, rightVel);
    }

    /**
     * Set angular velocity in degrees per second
     * Positive = counterclockwise, Negative = clockwise
     */
    public void setAngularVelocity(double degreesPerSecond) {
        double radiansPerSec = Math.toRadians(degreesPerSecond);
        double wheelVelocityInchesPerSec = radiansPerSec * (VelocityParams.TRACK_WIDTH_INCHES / 2.0);

        double wheelCircumference = VelocityParams.WHEEL_DIAMETER_INCHES * Math.PI;
        double ticksPerInch = VelocityParams.TICKS_PER_REV / wheelCircumference;
        double wheelVelocityTicksPerSec = wheelVelocityInchesPerSec * ticksPerInch;

        tankDriveVelocity(-wheelVelocityTicksPerSec, wheelVelocityTicksPerSec);
    }

    public double getLeftVelocity() {
        return (lf.getVelocity() + lb.getVelocity()) / 2.0;
    }

    public double getRightVelocity() {
        return (rf.getVelocity() + rb.getVelocity()) / 2.0;
    }

    // === ODOMETRY METHODS ===

    public void updateOdometry() {
        long currentTime = System.currentTimeMillis();

        // Update pinpoint sensor
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        // Get raw position data
        double rawX = pose.getX(DistanceUnit.INCH);
        double rawY = pose.getY(DistanceUnit.INCH);
        double rawHeading = pose.getHeading(AngleUnit.RADIANS);

        // Apply position smoothing to reduce noise
        if (lastOdometryUpdate > 0) {
            double deltaTime = (currentTime - lastOdometryUpdate) / 1000.0; // Convert to seconds

            // Smooth position updates
            robotX = lastRobotX + (rawX - lastRobotX) * OdometryParams.POSITION_SMOOTHING_FACTOR;
            robotY = lastRobotY + (rawY - lastRobotY) * OdometryParams.POSITION_SMOOTHING_FACTOR;

            // Calculate heading drift and apply correction
            double headingDelta = rawHeading - lastRobotHeading;
            // Normalize heading delta to -pi to pi
            while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
            while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;

            // Apply heading drift correction based on time elapsed
            double driftCorrection = imuDriftRate * deltaTime;
            robotHeading = lastRobotHeading + headingDelta - driftCorrection;
        } else {
            // First update - use raw values
            robotX = rawX;
            robotY = rawY;
            robotHeading = rawHeading;
        }

        // Normalize heading to -pi to pi
        while (robotHeading > Math.PI) robotHeading -= 2 * Math.PI;
        while (robotHeading < -Math.PI) robotHeading += 2 * Math.PI;

        // Store current values for next update
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotHeading = robotHeading;
        lastOdometryUpdate = currentTime;

        // Update IMU drift calibration periodically (every 5 seconds)
        if (currentTime - lastImuCalibration > 5000) {
            calibrateImuDrift();
            lastImuCalibration = currentTime;
        }
    }

    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
        initializeVelocityControl();
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;

        // Reset tracking variables
        lastRobotX = 0.0;
        lastRobotY = 0.0;
        lastRobotHeading = 0.0;
        lastOdometryUpdate = 0;
        imuDriftRate = 0.0;
        lastImuCalibration = System.currentTimeMillis();

        // Reset acceleration ramping
        currentDrivePower = 0.0;
        currentTurnPower = 0.0;
    }

    /**
     * Calibrate IMU drift rate by monitoring heading changes over time
     */
    private void calibrateImuDrift() {
        // This method can be enhanced to detect and correct systematic IMU drift
        // For now, we'll use a simple approach based on expected drift patterns
        if (lastOdometryUpdate > 0) {
            long timeDelta = System.currentTimeMillis() - lastOdometryUpdate;
            if (timeDelta > 0) {
                // Estimate drift rate based on typical IMU characteristics
                // This can be tuned based on observed behavior
                imuDriftRate = 0.001; // Small constant drift correction (rad/s)
            }
        }
    }

    private void setMotorsBrakeMode() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(double x, double y, double heading) {
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
        pinpoint.setPosition(newPose);
        robotX = x;
        robotY = y;
        robotHeading = heading;
    }

    // === SPEED MODE METHODS ===

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

    // === GETTERS (Required by OdometryTestTeleop) ===

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

    // Velocities (returned in INCHES per second for consistency)
    public double getVelocityX() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getVelocityY() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    // Pinpoint access methods
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

    // === TELEMETRY METHODS ===

    @SuppressLint("DefaultLocale")
    public void getMotorStatus() {
        if (linearOpMode != null) {
            linearOpMode.telemetry.addLine("=== DRIVE STATUS ===");
            linearOpMode.telemetry.addData("Mode", currentDriveMode);
            linearOpMode.telemetry.addData("Speed Mode", isFastSpeedMode ? "FAST" : "SLOW");
            linearOpMode.telemetry.addLine();
            linearOpMode.telemetry.addData("Front Left Power", String.format(Locale.US, "%.2f", lf.getPower()));
            linearOpMode.telemetry.addData("Front Right Power", String.format(Locale.US, "%.2f", rf.getPower()));
            linearOpMode.telemetry.addData("Back Left Power", String.format(Locale.US, "%.2f", lb.getPower()));
            linearOpMode.telemetry.addData("Back Right Power", String.format(Locale.US, "%.2f", rb.getPower()));

            if (currentDriveMode == DriveMode.VELOCITY) {
                linearOpMode.telemetry.addLine();
                linearOpMode.telemetry.addData("Left Avg Vel", String.format(Locale.US, "%.0f ticks/s", getLeftVelocity()));
                linearOpMode.telemetry.addData("Right Avg Vel", String.format(Locale.US, "%.0f ticks/s", getRightVelocity()));
            }

            // Debug IMU and odometry
            linearOpMode.telemetry.addLine();
            linearOpMode.telemetry.addData("X Position", String.format(Locale.US, "%.2f in", robotX));
            linearOpMode.telemetry.addData("Y Position", String.format(Locale.US, "%.2f in", robotY));
            linearOpMode.telemetry.addData("Heading (processed)", String.format(Locale.US, "%.2f°", Math.toDegrees(robotHeading)));
            linearOpMode.telemetry.addData("Yaw Scalar", OdometryParams.YAW_SCALAR);
            linearOpMode.telemetry.addData("IMU Drift Rate", String.format(Locale.US, "%.6f rad/s", imuDriftRate));

        } else if (iterativeOpMode != null) {
            iterativeOpMode.telemetry.addLine("=== DRIVE STATUS ===");
            iterativeOpMode.telemetry.addData("Mode", currentDriveMode);
            iterativeOpMode.telemetry.addData("Front Left", String.format(Locale.US, "%.2f", lf.getPower()));
            iterativeOpMode.telemetry.addData("Front Right", String.format(Locale.US, "%.2f", rf.getPower()));
            iterativeOpMode.telemetry.addData("Back Left", String.format(Locale.US, "%.2f", lb.getPower()));
            iterativeOpMode.telemetry.addData("Back Right", String.format(Locale.US, "%.2f", rb.getPower()));

            if (currentDriveMode == DriveMode.VELOCITY) {
                iterativeOpMode.telemetry.addLine();
                iterativeOpMode.telemetry.addData("Left Vel", String.format(Locale.US, "%.0f t/s", getLeftVelocity()));
                iterativeOpMode.telemetry.addData("Right Vel", String.format(Locale.US, "%.0f t/s", getRightVelocity()));
            }

            // Debug IMU and odometry
            iterativeOpMode.telemetry.addLine();
            iterativeOpMode.telemetry.addData("X Position", String.format(Locale.US, "%.2f in", robotX));
            iterativeOpMode.telemetry.addData("Y Position", String.format(Locale.US, "%.2f in", robotY));
            iterativeOpMode.telemetry.addData("Heading (processed)", String.format(Locale.US, "%.2f°", Math.toDegrees(robotHeading)));
            iterativeOpMode.telemetry.addData("Yaw Scalar", OdometryParams.YAW_SCALAR);
            iterativeOpMode.telemetry.addData("IMU Drift Rate", String.format(Locale.US, "%.6f rad/s", imuDriftRate));
        }
    }

    public String getMotorPowers() {
        return String.format(Locale.US, "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                lf.getPower(), rf.getPower(),
                lb.getPower(), rb.getPower());
    }

    public String getMotorVelocities() {
        return String.format(Locale.US, "FL:%.0f FR:%.0f BL:%.0f BR:%.0f",
                lf.getVelocity(), rf.getVelocity(),
                lb.getVelocity(), rb.getVelocity());
    }
}