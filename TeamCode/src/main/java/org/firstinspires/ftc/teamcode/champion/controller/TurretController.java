package org.firstinspires.ftc.teamcode.champion.controller;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class TurretController {
    public static String TURRET_SERVO_NAME = "turret";
    public static String TURRET_ANALOG_NAME = "turret_analog";
    // Constants for Axon mini servo with gear ratio
    public static double VOLTAGE_TO_DEGREES = 360.0 / 3.3;
    public static double GEAR_RATIO = 2.5;
    // Tuning parameters for turnToAngle
    public static double TURN_POWER = 1.0;
    public static double ANGLE_TOLERANCE = 2.0;
    public static double TIMEOUT_SECONDS = 5.0;
    private final CRServo turretServo;
    private final AnalogInput turretAnalog;
    private final ElapsedTime runtime;
    // Velocity tracking
    private double lastAngle = 0;
    private double lastVelocityTime = 0;
    private double velocity = 0;
    public TurretController(LinearOpMode opMode) {
        turretServo = opMode.hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);
        turretAnalog = opMode.hardwareMap.get(AnalogInput.class, TURRET_ANALOG_NAME);
        runtime = new ElapsedTime();
        lastAngle = getCurrentAngle();
        lastVelocityTime = runtime.seconds();
    }
    /**
     * Update method - handles velocity calculation
     * Should be called every loop for velocity tracking
     */
    public void update() {
        // Update velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastVelocityTime;
        if (deltaTime > 0.1) {
            double currentAngle = getCurrentAngle();
            double deltaAngle = currentAngle - lastAngle;
            // Handle wraparound
            if (deltaAngle > 180) deltaAngle -= 360;
            if (deltaAngle < -180) deltaAngle += 360;
            velocity = deltaAngle / deltaTime;
            lastAngle = currentAngle;
            lastVelocityTime = currentTime;
        }
    }
    /**
     * Turn the turret to a specified angle
     * Handles angles > 360 and negative angles by counting full rotations
     *
     * @param targetAngle The angle to turn to (can be > 360 or negative)
     * @return true if target reached, false if timeout occurred
     */
    public boolean turnToAngle(double targetAngle) {

        ElapsedTime turnTimer = new ElapsedTime();
        double finalPosition = (getCurrentAngle() + targetAngle) * GEAR_RATIO;
        // Now turn to final position
        while (!isNear(finalPosition, ANGLE_TOLERANCE)) {
            update();

            double currentAngle = getCurrentAngle();
            double error = (finalPosition - currentAngle);

            // Set power based on direction
            if (error > 0) {
                turretServo.setPower(TURN_POWER);
            } else {
                turretServo.setPower(-TURN_POWER);
            }

            // Timeout check
            if (turnTimer.seconds() > TIMEOUT_SECONDS) {
                stop();
                return false;
            }
        }

        stop();
        return true;
    }
    /**
     * Set continuous power to turret
     */
    public void setPower(double power) {
        turretServo.setPower(power);
    }
    /**
     * Stop the turret
     */
    public void stop() {
        turretServo.setPower(0);
    }
    /**
     * Get current angle from analog feedback (accounts for gear ratio)
     */
    public double getCurrentAngle() {
        double voltage = turretAnalog.getVoltage();
        double servoAngle = voltage * VOLTAGE_TO_DEGREES;
        return servoAngle / GEAR_RATIO;
    }
    /**
     * Get the current velocity in degrees per second
     */
    public double getVelocity() {
        return velocity;
    }
    /**
     * Get the raw voltage from analog feedback
     */
    public double getVoltage() {
        return turretAnalog.getVoltage();
    }
    /**
     * Get the servo angle (before gear ratio conversion)
     */
    public double getServoAngle() {
        return turretAnalog.getVoltage() * VOLTAGE_TO_DEGREES;
    }
    /**
     * Get the current servo power
     */
    public double getPower() {
        return turretServo.getPower();
    }
    /**
     * Check if turret is near a specified angle
     */
    public boolean isNear(double angle, double tolerance) {
        double currentAngle = getCurrentAngle();
        double error = angle - currentAngle;

        // Handle wraparound
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return Math.abs(error) < tolerance;
    }
}