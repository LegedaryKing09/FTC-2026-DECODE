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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
public class JohnController {

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

    // RPM Tracking for each motor
    private double motor1LeftRPM = 0.0;
    private double motor2LeftRPM = 0.0;
    private double motor3LeftRPM = 0.0;
    private double motor1RightRPM = 0.0;
    private double motor2RightRPM = 0.0;
    private double motor3RightRPM = 0.0;

    // RPM History for graphing (timestamp, rpm value)
    public static class RPMDataPoint {
        public long timestamp;
        public double rpm;

        public RPMDataPoint(long timestamp, double rpm) {
            this.timestamp = timestamp;
            this.rpm = rpm;
        }
    }

    // Store last N data points for each motor
    public static int MAX_DATA_POINTS = 500;  // Configurable history size
    private final List<RPMDataPoint> motor1LeftHistory = new ArrayList<>();
    private final List<RPMDataPoint> motor2LeftHistory = new ArrayList<>();
    private final List<RPMDataPoint> motor3LeftHistory = new ArrayList<>();
    private final List<RPMDataPoint> motor1RightHistory = new ArrayList<>();
    private final List<RPMDataPoint> motor2RightHistory = new ArrayList<>();
    private final List<RPMDataPoint> motor3RightHistory = new ArrayList<>();

    // Speed mode settings - ADJUSTED FOR BETTER CONTROL
    public static double FAST_SPEED_MULTIPLIER = 1;
    public static double FAST_TURN_MULTIPLIER = 1;
    public static double SLOW_SPEED_MULTIPLIER = 0.2;
    public static double SLOW_TURN_MULTIPLIER = 0.2;

    // Acceleration ramping to reduce inertia issues
    public static double ACCELERATION_RATE = 0.15;
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

        public static double VELOCITY_P = 26;
        public static double VELOCITY_I = 0.0;
        public static double VELOCITY_D = 0.1;
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

        public static double HEADING_DRIFT_CORRECTION = 0.02;
        public static long ODOMETRY_UPDATE_INTERVAL_MS = 10;
        public static double POSITION_SMOOTHING_FACTOR = 0.1;
    }

    // Constructor for LinearOpMode
    public JohnController(LinearOpMode opMode) {
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
        motor2Left.setDirection(DcMotor.Direction.REVERSE);
        motor3Left.setDirection(DcMotor.Direction.REVERSE);
        motor1Right.setDirection(DcMotor.Direction.FORWARD);
        motor2Right.setDirection(DcMotor.Direction.FORWARD);
        motor3Right.setDirection(DcMotor.Direction.FORWARD);

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

    // === RPM TRACKING METHODS ===

    /**
     * Update RPM readings for all motors and store in history
     * Call this in your main loop for continuous tracking
     */
    public void updateRPM() {
        long currentTime = System.currentTimeMillis();

        // Calculate RPM from velocity (ticks per second to RPM)
        motor1LeftRPM = ticksPerSecondToRPM(motor1Left.getVelocity());
        motor2LeftRPM = ticksPerSecondToRPM(motor2Left.getVelocity());
        motor3LeftRPM = ticksPerSecondToRPM(motor3Left.getVelocity());
        motor1RightRPM = ticksPerSecondToRPM(motor1Right.getVelocity());
        motor2RightRPM = ticksPerSecondToRPM(motor2Right.getVelocity());
        motor3RightRPM = ticksPerSecondToRPM(motor3Right.getVelocity());

        // Add to history
        addRPMDataPoint(motor1LeftHistory, currentTime, motor1LeftRPM);
        addRPMDataPoint(motor2LeftHistory, currentTime, motor2LeftRPM);
        addRPMDataPoint(motor3LeftHistory, currentTime, motor3LeftRPM);
        addRPMDataPoint(motor1RightHistory, currentTime, motor1RightRPM);
        addRPMDataPoint(motor2RightHistory, currentTime, motor2RightRPM);
        addRPMDataPoint(motor3RightHistory, currentTime, motor3RightRPM);
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / VelocityParams.TICKS_PER_REV) * 60.0;
    }

    private void addRPMDataPoint(List<RPMDataPoint> history, long timestamp, double rpm) {
        history.add(new RPMDataPoint(timestamp, rpm));

        // Keep only last MAX_DATA_POINTS
        if (history.size() > MAX_DATA_POINTS) {
            history.remove(0);
        }
    }

    /**
     * Clear all RPM history data
     */
    public void clearRPMHistory() {
        motor1LeftHistory.clear();
        motor2LeftHistory.clear();
        motor3LeftHistory.clear();
        motor1RightHistory.clear();
        motor2RightHistory.clear();
        motor3RightHistory.clear();
    }

    // === RPM GETTERS ===

    public double getMotor1LeftRPM() {
        return motor1LeftRPM;
    }

    public double getMotor2LeftRPM() {
        return motor2LeftRPM;
    }

    public double getMotor3LeftRPM() {
        return motor3LeftRPM;
    }

    public double getMotor1RightRPM() {
        return motor1RightRPM;
    }

    public double getMotor2RightRPM() {
        return motor2RightRPM;
    }

    public double getMotor3RightRPM() {
        return motor3RightRPM;
    }

    public double getLeftAverageRPM() {
        return (motor1LeftRPM + motor2LeftRPM + motor3LeftRPM) / 3.0;
    }

    public double getRightAverageRPM() {
        return (motor1RightRPM + motor2RightRPM + motor3RightRPM) / 3.0;
    }

    // === RPM HISTORY GETTERS (for graphing) ===

    public List<RPMDataPoint> getMotor1LeftHistory() {
        return new ArrayList<>(motor1LeftHistory);
    }

    public List<RPMDataPoint> getMotor2LeftHistory() {
        return new ArrayList<>(motor2LeftHistory);
    }

    public List<RPMDataPoint> getMotor3LeftHistory() {
        return new ArrayList<>(motor3LeftHistory);
    }

    public List<RPMDataPoint> getMotor1RightHistory() {
        return new ArrayList<>(motor1RightHistory);
    }

    public List<RPMDataPoint> getMotor2RightHistory() {
        return new ArrayList<>(motor2RightHistory);
    }

    public List<RPMDataPoint> getMotor3RightHistory() {
        return new ArrayList<>(motor3RightHistory);
    }

    // === DRIVE CONTROL METHODS ===

    public void setDriveMode(DriveMode mode) {
        this.currentDriveMode = mode;
    }

    public DriveMode getCurrentDriveMode() {
        return currentDriveMode;
    }

    public void arcadeDrive(double drive, double turn) {
        double targetDrivePower = drive;
        double targetTurnPower = turn;

        if (Math.abs(targetDrivePower - currentDrivePower) > ACCELERATION_RATE) {
            if (targetDrivePower > currentDrivePower) {
                currentDrivePower += ACCELERATION_RATE;
            } else {
                currentDrivePower -= ACCELERATION_RATE;
            }
        } else {
            currentDrivePower = targetDrivePower;
        }

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
        double velocityScale = 0.8;
        double leftVel = leftPower * VelocityParams.MAX_TICKS_PER_SEC * velocityScale;
        double rightVel = rightPower * VelocityParams.MAX_TICKS_PER_SEC * velocityScale;
        tankDriveVelocity(leftVel, rightVel);
    }

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

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        double rawX = pose.getX(DistanceUnit.INCH);
        double rawY = pose.getY(DistanceUnit.INCH);
        double rawHeading = pose.getHeading(AngleUnit.RADIANS);

        if (lastOdometryUpdate > 0) {
            double deltaTime = (currentTime - lastOdometryUpdate) / 1000.0;

            robotX = lastRobotX + (rawX - lastRobotX) * OdometryParams.POSITION_SMOOTHING_FACTOR;
            robotY = lastRobotY + (rawY - lastRobotY) * OdometryParams.POSITION_SMOOTHING_FACTOR;

            double headingDelta = rawHeading - lastRobotHeading;
            while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
            while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;

            double driftCorrection = imuDriftRate * deltaTime;
            robotHeading = lastRobotHeading + headingDelta - driftCorrection;
        } else {
            robotX = rawX;
            robotY = rawY;
            robotHeading = rawHeading;
        }

        while (robotHeading > Math.PI) robotHeading -= 2 * Math.PI;
        while (robotHeading < -Math.PI) robotHeading += 2 * Math.PI;

        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotHeading = robotHeading;
        lastOdometryUpdate = currentTime;

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

        lastRobotX = 0.0;
        lastRobotY = 0.0;
        lastRobotHeading = 0.0;
        lastOdometryUpdate = 0;
        imuDriftRate = 0.0;
        lastImuCalibration = System.currentTimeMillis();

        currentDrivePower = 0.0;
        currentTurnPower = 0.0;

        clearRPMHistory();
    }

    private void calibrateImuDrift() {
        if (lastOdometryUpdate > 0) {
            long timeDelta = System.currentTimeMillis() - lastOdometryUpdate;
            if (timeDelta > 0) {
                imuDriftRate = 0.001;
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

    public int getXOdoPosition() {
        return pinpoint.getEncoderX();
    }

    public int getYOdoPosition() {
        return pinpoint.getEncoderY();
    }

    public double getVelocityX() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getVelocityY() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
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

    // === TELEMETRY METHODS ===

    @SuppressLint("DefaultLocale")
    public void getMotorStatus() {
        if (linearOpMode != null) {
            linearOpMode.telemetry.addLine("=== DRIVE STATUS ===");
            linearOpMode.telemetry.addData("Mode", currentDriveMode);
            linearOpMode.telemetry.addData("Speed Mode", isFastSpeedMode ? "FAST" : "SLOW");
            linearOpMode.telemetry.addLine();

            // RPM Display
            linearOpMode.telemetry.addLine("=== MOTOR RPM ===");
            linearOpMode.telemetry.addData("L1 RPM", String.format(Locale.US, "%.1f", motor1LeftRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("L2 RPM", String.format(Locale.US, "%.1f", motor2LeftRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("L3 RPM", String.format(Locale.US, "%.1f", motor3LeftRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("R1 RPM", String.format(Locale.US, "%.1f", motor1RightRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("R2 RPM", String.format(Locale.US, "%.1f", motor2RightRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("R3 RPM", String.format(Locale.US, "%.1f", motor3RightRPM * (28 / 751.8)));
            linearOpMode.telemetry.addData("Left Avg RPM", String.format(Locale.US, "%.1f", getLeftAverageRPM()));
            linearOpMode.telemetry.addData("Right Avg RPM", String.format(Locale.US, "%.1f", getRightAverageRPM()));

            if (currentDriveMode == DriveMode.VELOCITY) {
                linearOpMode.telemetry.addLine();
                linearOpMode.telemetry.addData("Left Avg Vel", String.format(Locale.US, "%.0f ticks/s", getLeftVelocity()));
                linearOpMode.telemetry.addData("Right Avg Vel", String.format(Locale.US, "%.0f ticks/s", getRightVelocity()));
            }

            linearOpMode.telemetry.addLine();
            linearOpMode.telemetry.addData("X Position", String.format(Locale.US, "%.2f in", robotX));
            linearOpMode.telemetry.addData("Y Position", String.format(Locale.US, "%.2f in", robotY));
            // Add this to complete the getMotorStatus() method in JohnController:

            linearOpMode.telemetry.addData("Heading (processed)", String.format(Locale.US, "%.2f°", Math.toDegrees(robotHeading)));
            linearOpMode.telemetry.addData("Yaw Scalar", OdometryParams.YAW_SCALAR);
            linearOpMode.telemetry.addData("IMU Drift Rate", String.format(Locale.US, "%.6f rad/s", imuDriftRate));

        } else if (iterativeOpMode != null) {
            iterativeOpMode.telemetry.addLine("=== DRIVE STATUS ===");
            iterativeOpMode.telemetry.addData("Mode", currentDriveMode);
            iterativeOpMode.telemetry.addData("Speed Mode", isFastSpeedMode ? "FAST" : "SLOW");
            iterativeOpMode.telemetry.addLine();

            // RPM Display
            iterativeOpMode.telemetry.addLine("=== MOTOR RPM ===");
            iterativeOpMode.telemetry.addData("L1 RPM", String.format(Locale.US, "%.1f", motor1LeftRPM));
            iterativeOpMode.telemetry.addData("L2 RPM", String.format(Locale.US, "%.1f", motor2LeftRPM));
            iterativeOpMode.telemetry.addData("L3 RPM", String.format(Locale.US, "%.1f", motor3LeftRPM));
            iterativeOpMode.telemetry.addData("R1 RPM", String.format(Locale.US, "%.1f", motor1RightRPM));
            iterativeOpMode.telemetry.addData("R2 RPM", String.format(Locale.US, "%.1f", motor2RightRPM));
            iterativeOpMode.telemetry.addData("R3 RPM", String.format(Locale.US, "%.1f", motor3RightRPM));
            iterativeOpMode.telemetry.addData("Left Avg RPM", String.format(Locale.US, "%.1f", getLeftAverageRPM()));
            iterativeOpMode.telemetry.addData("Right Avg RPM", String.format(Locale.US, "%.1f", getRightAverageRPM()));

            if (currentDriveMode == DriveMode.VELOCITY) {
                iterativeOpMode.telemetry.addLine();
                iterativeOpMode.telemetry.addData("Left Vel", String.format(Locale.US, "%.0f t/s", getLeftVelocity()));
                iterativeOpMode.telemetry.addData("Right Vel", String.format(Locale.US, "%.0f t/s", getRightVelocity()));
            }

            iterativeOpMode.telemetry.addLine();
            iterativeOpMode.telemetry.addData("X Position", String.format(Locale.US, "%.2f in", robotX));
            iterativeOpMode.telemetry.addData("Y Position", String.format(Locale.US, "%.2f in", robotY));
            iterativeOpMode.telemetry.addData("Heading (processed)", String.format(Locale.US, "%.2f°", Math.toDegrees(robotHeading)));
            iterativeOpMode.telemetry.addData("Yaw Scalar", OdometryParams.YAW_SCALAR);
            iterativeOpMode.telemetry.addData("IMU Drift Rate", String.format(Locale.US, "%.6f rad/s", imuDriftRate));
        }
    }

    public String getMotorPowers() {
        return String.format(Locale.US, "L1:%.2f L2:%.2f L3:%.2f R1:%.2f R2:%.2f R3:%.2f",
                motor1Left.getPower(), motor2Left.getPower(), motor3Left.getPower(),
                motor1Right.getPower(), motor2Right.getPower(), motor3Right.getPower());
    }

    public String getMotorVelocities() {
        return String.format(Locale.US, "L1:%.0f L2:%.0f L3:%.0f R1:%.0f R2:%.0f R3:%.0f",
                motor1Left.getVelocity(), motor2Left.getVelocity(), motor3Left.getVelocity(),
                motor1Right.getVelocity(), motor2Right.getVelocity(), motor3Right.getVelocity());
    }
}