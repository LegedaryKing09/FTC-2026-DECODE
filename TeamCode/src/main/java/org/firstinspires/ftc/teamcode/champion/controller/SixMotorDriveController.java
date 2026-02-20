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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@Config
public class SixMotorDriveController {

    // Motor configuration names
    public static String L1_NAME = "l1";
    public static String L2_NAME = "l2";
    public static String L3_NAME = "l3";
    public static String R1_NAME = "r1";
    public static String R2_NAME = "r2";
    public static String R3_NAME = "r3";

    private final DcMotorEx motor1Left, motor2Left, motor3Left, motor1Right, motor2Right, motor3Right;

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
    public static double FAST_SPEED_MULTIPLIER = 0.4;  // Reduced from 0.3
    public static double FAST_TURN_MULTIPLIER = 0.4;    // Significantly reduced from 1.5
    public static double SLOW_SPEED_MULTIPLIER = 0.2;
    public static double SLOW_TURN_MULTIPLIER = 0.2;    // Significantly reduced from 0.8

    // Acceleration ramping to reduce inertia issues
    public static double ACCELERATION_RATE = 0.15;  // How quickly to ramp up/down power
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

        // TUNABLE PIDF - Reduced P and D for smoother response
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

        // IMU and odometry accuracy improvements
        public static double HEADING_DRIFT_CORRECTION = 0.02;
        public static long ODOMETRY_UPDATE_INTERVAL_MS = 10;
        public static double POSITION_SMOOTHING_FACTOR = 0.1;
    }

    // Constructor for LinearOpMode
    public SixMotorDriveController(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        this.iterativeOpMode = null;

        motor1Left = opMode.hardwareMap.get(DcMotorEx.class, L1_NAME);
        motor2Left = opMode.hardwareMap.get(DcMotorEx.class, L2_NAME);
        motor3Left = opMode.hardwareMap.get(DcMotorEx.class, L3_NAME);
        motor1Right = opMode.hardwareMap.get(DcMotorEx.class, R1_NAME);
        motor2Right = opMode.hardwareMap.get(DcMotorEx.class, R2_NAME);
        motor3Right = opMode.hardwareMap.get(DcMotorEx.class, R3_NAME);

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        motor1Left.setDirection(DcMotor.Direction.FORWARD);
        motor2Left.setDirection(DcMotor.Direction.FORWARD);
        motor3Left.setDirection(DcMotor.Direction.FORWARD);
        motor1Right.setDirection(DcMotor.Direction.REVERSE);
        motor2Right.setDirection(DcMotor.Direction.REVERSE);
        motor3Right.setDirection(DcMotor.Direction.REVERSE);


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
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        } else {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        }

        pinpoint.setOffsets(OdometryParams.X_OFFSET_MM, OdometryParams.Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setYawScalar(OdometryParams.YAW_SCALAR);
    }

    private void initializeVelocityControl() {
        motor1Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPIDF(motor1Left);
        setMotorPIDF(motor2Left);
        setMotorPIDF(motor3Left);
        setMotorPIDF(motor1Right);
        setMotorPIDF(motor2Right);
        setMotorPIDF(motor3Right);
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
        double targetDrivePower = drive;
        double targetTurnPower = turn;

        // Smooth the drive power changes
        if (Math.abs(targetDrivePower - currentDrivePower) > ACCELERATION_RATE) {
            if (targetDrivePower > currentDrivePower) {
                currentDrivePower += ACCELERATION_RATE;
            } else {
                currentDrivePower -= ACCELERATION_RATE;
            }
        } else {
            currentDrivePower = targetDrivePower;
        }

        // Smooth the turn power changes
        if (Math.abs(targetTurnPower - currentTurnPower) > ACCELERATION_RATE) {
            if (targetTurnPower > currentTurnPower) {
                currentTurnPower += ACCELERATION_RATE;
            } else {
                currentTurnPower -= ACCELERATION_RATE;
            }
        } else {
            currentTurnPower = targetTurnPower;
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
        motor1Left.setPower(leftPower);
        motor2Left.setPower(leftPower);
        motor3Left.setPower(leftPower);
        motor1Right.setPower(rightPower);
        motor2Right.setPower(rightPower);
        motor3Right.setPower(rightPower);
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

        motor1Left.setVelocity(leftVelocity);
        motor2Left.setVelocity(leftVelocity);
        motor3Left.setVelocity(leftVelocity);
        motor1Right.setVelocity(rightVelocity);
        motor2Right.setVelocity(rightVelocity);
        motor3Right.setVelocity(rightVelocity);
    }

    public void tankDriveVelocityNormalized(double leftPower, double rightPower) {
        // Apply additional scaling for smoother velocity control
        double velocityScale = 0.8;  // Reduce max velocity for better control
        double leftVel = leftPower * VelocityParams.MAX_TICKS_PER_SEC * velocityScale;
        double rightVel = rightPower * VelocityParams.MAX_TICKS_PER_SEC * velocityScale;
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
        return (motor1Left.getVelocity() + motor2Left.getVelocity() + motor3Left.getVelocity()) / 3.0;
    }

    public double getRightVelocity() {
        return (motor1Right.getVelocity() + motor2Right.getVelocity() + motor3Right.getVelocity()) / 3.0;
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
        motor1Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsPowerZero() {
        motor1Left.setPower(0);
        motor2Left.setPower(0);
        motor3Left.setPower(0);
        motor1Right.setPower(0);
        motor2Right.setPower(0);
        motor3Right.setPower(0);
    }

    public void setL1MotorsPowerOne() {
        motor1Left.setPower(1);
    }

    public void setL2MotorsPowerOne() {
        motor2Left.setPower(1);
    }

    public void setL3MotorsPowerOne() {
        motor3Left.setPower(1);
    }

    public void setR1MotorsPowerOne() {
        motor1Right.setPower(1);
    }

    public void setR2MotorsPowerOne() {
        motor2Right.setPower(1);
    }

    public void setR3MotorsPowerOne() {
        motor3Right.setPower(1);
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

    // === GETTERS ===

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
//            linearOpMode.telemetry.addData("Front Left Power", String.format(Locale.US, "%.2f", motor1Left.getPower()));
//            linearOpMode.telemetry.addData("Front Right Power", String.format(Locale.US, "%.2f", frontRight.getPower()));
//            linearOpMode.telemetry.addData("Back Left Power", String.format(Locale.US, "%.2f", backLeft.getPower()));
//            linearOpMode.telemetry.addData("Back Right Power", String.format(Locale.US, "%.2f", backRight.getPower()));

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
//            iterativeOpMode.telemetry.addData("Front Left", String.format(Locale.US, "%.2f", motor1Left.getPower()));
//            iterativeOpMode.telemetry.addData("Front Right", String.format(Locale.US, "%.2f", frontRight.getPower()));
//            iterativeOpMode.telemetry.addData("Back Left", String.format(Locale.US, "%.2f", backLeft.getPower()));
//            iterativeOpMode.telemetry.addData("Back Right", String.format(Locale.US, "%.2f", backRight.getPower()));

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
                motor1Left.getPower(), motor2Left.getPower(), motor3Left.getPower(),
                motor1Right.getPower(), motor2Right.getPower(), motor3Right.getPower());
    }

    public String getMotorVelocities() {
        return String.format(Locale.US, "FL:%.0f FR:%.0f BL:%.0f BR:%.0f",
                motor1Left.getVelocity(), motor2Left.getVelocity(), motor3Left.getVelocity(),
                motor1Right.getVelocity(), motor2Right.getVelocity(), motor3Right.getVelocity());
    }
}