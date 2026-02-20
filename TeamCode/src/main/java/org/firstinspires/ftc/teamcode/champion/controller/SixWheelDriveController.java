package org.firstinspires.ftc.teamcode.champion.controller;

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

    // Acceleration ramping
    public static double ACCELERATION_RATE = 0.15;
    private double currentDrivePower = 0.0;
    private double currentTurnPower = 0.0;

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

        // PIDF Coefficients (tuned with VelocityPIDFTuner)
        public static double VELOCITY_P = 2.0;
        public static double VELOCITY_I = 0.0;
        public static double VELOCITY_D = 0.5;
        public static double VELOCITY_F = 13.0;

        // === ENCODER CONFIGURATION ===
        // Which motor to use for encoder reading on each side
        public static boolean USE_FRONT_ENCODER_LEFT = true;   // true = LF, false = LB
        public static boolean USE_FRONT_ENCODER_RIGHT = true;  // true = RF, false = RB

        // Invert encoder readings if needed (set to true if reading is negated)
        public static boolean INVERT_LEFT_ENCODER = false;
        public static boolean INVERT_RIGHT_ENCODER = true;
    }
    public double YAW_SCALAR = -1.0;      // Tune
    @Config
    public static class OdometryParams {
        public static boolean USE_4_BAR_PODS = true;
        public static double ENCODER_RESOLUTION_MM = 19.89;  // mm per tick for 4-bar pods
        public static double X_OFFSET_MM = -101.6;   // Forward pod offset
        public static double Y_OFFSET_MM = -158.75;  // Strafe pod offset
        public static boolean X_ENCODER_REVERSED = false;
        public static boolean Y_ENCODER_REVERSED = false;

        // === COORDINATE TRANSFORMATION ===
        // Fix axis mismatch between odometry and field coordinates
        // Correct field X = -rawY (from odometry)
        // Correct field Y = rawX (from odometry)
        public static boolean SWAP_XY = true;           // Swap X and Y axes
        public static boolean NEGATE_X = false;          // Negate the X value after swap
        public static boolean NEGATE_Y = true;         // Negate the Y value after swap
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

        initializeMotors();
        configurePinpoint();
        initializeVelocityControl();
        // NOTE: resetOdometry() removed - call manually if needed
        // This allows teleop to restore position from auton
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
        // NOTE: resetOdometry() removed - call manually if needed
        // This allows teleop to restore position from auton
    }

    private void initializeMotors() {
        // Set directions (left side forward, right side reversed)
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

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
        pinpoint.setYawScalar(YAW_SCALAR);
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

        // Get raw values from pinpoint
        double rawX = pose.getX(DistanceUnit.INCH);
        double rawY = pose.getY(DistanceUnit.INCH);

        // Apply coordinate transformation
        if (OdometryParams.SWAP_XY) {
            // After swap: X becomes Y, Y becomes X
            double tempX = rawY;  // new X = old Y
            double tempY = rawX;  // new Y = old X
            rawX = tempX;
            rawY = tempY;
        }

        if (OdometryParams.NEGATE_X) {
            rawX = -rawX;
        }

        if (OdometryParams.NEGATE_Y) {
            rawY = -rawY;
        }

        robotX = rawX;
        robotY = rawY;
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
        // Apply inverse transformation to convert field coords to pinpoint coords
        double pinpointX = x;
        double pinpointY = y;

        // Inverse of: if NEGATE_X then negate
        if (OdometryParams.NEGATE_X) {
            pinpointX = -pinpointX;
        }

        if (OdometryParams.NEGATE_Y) {
            pinpointY = -pinpointY;
        }

        // Inverse of swap: swap back
        if (OdometryParams.SWAP_XY) {
            double temp = pinpointX;
            pinpointX = pinpointY;
            pinpointY = temp;
        }

        Pose2D newPose = new Pose2D(DistanceUnit.INCH, pinpointX, pinpointY, AngleUnit.RADIANS, headingRadians);
        pinpoint.setPosition(newPose);

        // Store the field coordinates (what user expects)
        robotX = x;
        robotY = y;
        robotHeading = headingRadians;
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

    public double getVelocityX() {
        // With SWAP_XY, field-X comes from odometry Y
        double v = OdometryParams.SWAP_XY
                ? pinpoint.getVelY(DistanceUnit.INCH)
                : pinpoint.getVelX(DistanceUnit.INCH);
        return OdometryParams.NEGATE_X ? -v : v;
    }

    public double getVelocityY() {
        // With SWAP_XY, field-Y comes from odometry X
        double v = OdometryParams.SWAP_XY
                ? pinpoint.getVelX(DistanceUnit.INCH)
                : pinpoint.getVelY(DistanceUnit.INCH);
        return OdometryParams.NEGATE_Y ? -v : v;
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

}