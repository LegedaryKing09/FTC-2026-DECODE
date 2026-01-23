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

    // Speed mode settings
    public static double FAST_SPEED_MULTIPLIER = 1;
    public static double FAST_TURN_MULTIPLIER = 0.7;
    public static double SLOW_SPEED_MULTIPLIER = 0.5;
    public static double SLOW_TURN_MULTIPLIER = 0.4;

    // Acceleration ramping
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
        // Motor specs (goBILDA 312 RPM)
        public static double TICKS_PER_REV = 751.8;
        public static double MAX_RPM = 312.0;
        public static double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;  // ~3909

        // Wheel specs - CORRECTED
        public static double WHEEL_DIAMETER_MM = 68.5;
        public static double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;  // ~2.697 inches
        public static double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;  // ~8.47 inches
        public static double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES;  // ~88.7

        // Robot dimensions
        public static double TRACK_WIDTH_INCHES = 11.5;

        // PIDF Coefficients (tune with VelocityPIDTuner)
        public static double VELOCITY_P = 15.0;
        public static double VELOCITY_I = 0.0;
        public static double VELOCITY_D = 0.5;
        public static double VELOCITY_F = 12.0;

        // === ENCODER CONFIGURATION ===
        // Which motor to use for encoder reading on each side
        public static boolean USE_FRONT_ENCODER_LEFT = true;   // true = LF, false = LB
        public static boolean USE_FRONT_ENCODER_RIGHT = true;  // true = RF, false = RB

        // Invert encoder readings if needed (set to true if reading is negated)
        public static boolean INVERT_LEFT_ENCODER = false;
        public static boolean INVERT_RIGHT_ENCODER = true;
    }

    @Config
    public static class OdometryParams {
        public static boolean USE_4_BAR_PODS = true;
        public static double ENCODER_RESOLUTION_MM = 19.89;  // mm per tick for 4-bar pods
        public static double X_OFFSET_MM = -84.0;   // Forward pod offset
        public static double Y_OFFSET_MM = -168.0;  // Strafe pod offset
        public static double YAW_SCALAR = 1.0;      // Tune with OdometryTuner
        public static boolean X_ENCODER_REVERSED = false;
        public static boolean Y_ENCODER_REVERSED = false;
    }

    // === DRIVE ENCODER CONFIGURATION ===
    // Which motor encoder to use for velocity reading (front or back)
    public static boolean USE_FRONT_ENCODERS = true;  // true = LF/RF, false = LB/RB
    // Invert encoder readings if they're negated (change these if readings are wrong)
    public static boolean INVERT_LEFT_ENCODER = false;
    public static boolean INVERT_RIGHT_ENCODER = false;

    // Constructor for LinearOpMode
    public SixWheelDriveController(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        this.iterativeOpMode = null;

        lf = opMode.hardwareMap.get(DcMotorEx.class, LF_NAME);
        rf = opMode.hardwareMap.get(DcMotorEx.class, RF_NAME);
        lb = opMode.hardwareMap.get(DcMotorEx.class, LB_NAME);
        rb = opMode.hardwareMap.get(DcMotorEx.class, RB_NAME);

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        initializeMotors();
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

        initializeMotors();
        configurePinpoint();
        initializeVelocityControl();
        resetOdometry();
    }

    private void initializeMotors() {
        // Set directions (left side forward, right side reversed)
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configurePinpoint() {
        // Set encoder directions
        GoBildaPinpointDriver.EncoderDirection xDir = OdometryParams.X_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaPinpointDriver.EncoderDirection yDir = OdometryParams.Y_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        pinpoint.setEncoderDirections(xDir, yDir);

        // Set encoder resolution
        if (OdometryParams.USE_4_BAR_PODS) {
            pinpoint.setEncoderResolution((float) OdometryParams.ENCODER_RESOLUTION_MM, DistanceUnit.MM);
        } else {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        }

        // Set pod offsets from robot center
        pinpoint.setOffsets(OdometryParams.X_OFFSET_MM, OdometryParams.Y_OFFSET_MM, DistanceUnit.MM);

        // Set yaw scalar for heading accuracy
        pinpoint.setYawScalar(OdometryParams.YAW_SCALAR);
    }

    private void initializeVelocityControl() {
        // Reset encoders
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to velocity mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF coefficients
        applyVelocityPIDF();
    }

    private void applyVelocityPIDF() {
        lf.setVelocityPIDFCoefficients(
                VelocityParams.VELOCITY_P,
                VelocityParams.VELOCITY_I,
                VelocityParams.VELOCITY_D,
                VelocityParams.VELOCITY_F
        );
        rf.setVelocityPIDFCoefficients(
                VelocityParams.VELOCITY_P,
                VelocityParams.VELOCITY_I,
                VelocityParams.VELOCITY_D,
                VelocityParams.VELOCITY_F
        );
        lb.setVelocityPIDFCoefficients(
                VelocityParams.VELOCITY_P,
                VelocityParams.VELOCITY_I,
                VelocityParams.VELOCITY_D,
                VelocityParams.VELOCITY_F
        );
        rb.setVelocityPIDFCoefficients(
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

    /**
     * Arcade drive with acceleration ramping
     * @param drive Forward/backward (-1 to 1)
     * @param turn Left/right turn (-1 to 1)
     */
    public void arcadeDrive(double drive, double turn) {
        // Smooth acceleration
        currentDrivePower = rampValue(currentDrivePower, drive, ACCELERATION_RATE);
        currentTurnPower = rampValue(currentTurnPower, turn, ACCELERATION_RATE);

        if (currentDriveMode == DriveMode.VELOCITY) {
            arcadeDriveVelocity(currentDrivePower, currentTurnPower);
        } else {
            arcadeDrivePower(currentDrivePower, currentTurnPower);
        }
    }

    private double rampValue(double current, double target, double rate) {
        double diff = target - current;
        if (Math.abs(diff) <= rate) {
            return target;
        }
        return current + Math.copySign(rate, diff);
    }

    private void arcadeDrivePower(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        tankDrive(leftPower, rightPower);
    }

    private void arcadeDriveVelocity(double drive, double turn) {
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize if over 1.0
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
        currentDrivePower = 0;
        currentTurnPower = 0;

        if (currentDriveMode == DriveMode.VELOCITY) {
            tankDriveVelocity(0, 0);
        } else {
            tankDrive(0, 0);
        }
    }

    // === VELOCITY CONTROL METHODS ===

    /**
     * Set velocity in ticks per second
     */
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

    /**
     * Set velocity as normalized power (-1 to 1)
     */
    public void tankDriveVelocityNormalized(double leftPower, double rightPower) {
        double leftVel = leftPower * VelocityParams.MAX_TICKS_PER_SEC;
        double rightVel = rightPower * VelocityParams.MAX_TICKS_PER_SEC;
        tankDriveVelocity(leftVel, rightVel);
    }

    /**
     * Set velocity in inches per second
     */
    public void tankDriveInchesPerSecond(double leftIPS, double rightIPS) {
        double leftVel = leftIPS * VelocityParams.TICKS_PER_INCH;
        double rightVel = rightIPS * VelocityParams.TICKS_PER_INCH;
        tankDriveVelocity(leftVel, rightVel);
    }

    /**
     * Set angular velocity in degrees per second
     * Positive = counterclockwise
     */
    public void setAngularVelocity(double degreesPerSecond) {
        double radiansPerSec = Math.toRadians(degreesPerSecond);
        double wheelVelocityInchesPerSec = radiansPerSec * (VelocityParams.TRACK_WIDTH_INCHES / 2.0);
        double wheelVelocityTicksPerSec = wheelVelocityInchesPerSec * VelocityParams.TICKS_PER_INCH;

        // Left negative, right positive for CCW rotation
        tankDriveVelocity(-wheelVelocityTicksPerSec, wheelVelocityTicksPerSec);
    }

    public double getLeftVelocity() {
        // Use single encoder based on configuration
        double velocity = VelocityParams.USE_FRONT_ENCODER_LEFT ?
                lf.getVelocity() : lb.getVelocity();

        // Apply inversion if needed
        return VelocityParams.INVERT_LEFT_ENCODER ? -velocity : velocity;
    }

    public double getRightVelocity() {
        // Use single encoder based on configuration
        double velocity = VelocityParams.USE_FRONT_ENCODER_RIGHT ?
                rf.getVelocity() : rb.getVelocity();

        // Apply inversion if needed
        return VelocityParams.INVERT_RIGHT_ENCODER ? -velocity : velocity;
    }

    /**
     * Get robot speed in inches per second
     */
    public double getSpeedInchesPerSec() {
        double avgVel = (getLeftVelocity() + getRightVelocity()) / 2.0;
        return avgVel / VelocityParams.TICKS_PER_INCH;
    }

    // === ODOMETRY METHODS ===

    public void updateOdometry() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeading = pose.getHeading(AngleUnit.RADIANS);
    }

    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
        currentDrivePower = 0.0;
        currentTurnPower = 0.0;
    }

    public void setPosition(double x, double y, double headingRadians) {
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRadians);
        pinpoint.setPosition(newPose);
        robotX = x;
        robotY = y;
        robotHeading = headingRadians;
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

    // === TELEMETRY ===

    @SuppressLint("DefaultLocale")
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== DRIVE STATUS ===");
        telemetry.addData("Mode", currentDriveMode);
        telemetry.addData("Speed", isFastSpeedMode ? "FAST" : "SLOW");
        telemetry.addLine();

        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.2f in", robotX);
        telemetry.addData("Y", "%.2f in", robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
        telemetry.addLine();

        telemetry.addLine("=== VELOCITY ===");
        telemetry.addData("Left", "%.0f t/s", getLeftVelocity());
        telemetry.addData("Right", "%.0f t/s", getRightVelocity());
        telemetry.addData("Speed", "%.1f in/s", getSpeedInchesPerSec());
    }

    // Legacy method for compatibility
    @SuppressLint("DefaultLocale")
    public void getMotorStatus() {
        if (linearOpMode != null) {
            addTelemetry(linearOpMode.telemetry);
        } else if (iterativeOpMode != null) {
            addTelemetry(iterativeOpMode.telemetry);
        }
    }

    public String getMotorPowers() {
        return String.format(Locale.US, "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                lf.getPower(), rf.getPower(), lb.getPower(), rb.getPower());
    }

    public String getMotorVelocities() {
        return String.format(Locale.US, "FL:%.0f FR:%.0f BL:%.0f BR:%.0f",
                lf.getVelocity(), rf.getVelocity(), lb.getVelocity(), rb.getVelocity());
    }
}