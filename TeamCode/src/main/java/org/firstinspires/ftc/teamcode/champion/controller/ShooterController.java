package org.firstinspires.ftc.teamcode.champion.controller;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterController {
    public static String SHOOTER_NAME1 = "shooter1";
    public static String SHOOTER_NAME2 = "shooter2";

    // Target RPM values instead of power values
    public static double SHOOTER_FULL_RPM = 3000;
    public static double SHOOTER_HALF_RPM = 1500;
    public static double SHOOTER_QUARTER_RPM = 750;
    public static double SHOOTER_STOP_RPM = 0;

    // Motor specifications
    public static double TICKS_PER_REV = 28;  // Adjust based on your encoder
    public static double WHEEL_DIAMETER_METERS = 0.10795;

    // Velocity control tolerances
    public static double RPM_TOLERANCE = 50; // RPM tolerance for "at target" check

    private enum ShooterMode {
        VELOCITY_CONTROL, STOP
    }

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private ShooterMode shooterMode = ShooterMode.STOP;
    private double targetRPM = 0;

    public ShooterController(LinearOpMode opMode) {
        shooter1 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME1);
        shooter2 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Only shooter1 has encoder - set up for velocity control
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Shooter2 runs without encoder
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Get raw encoder position from shooter1
    public int getEncoderPosition() {
        return shooter1.getCurrentPosition();
    }

    // Get raw encoder velocity from shooter1 (ticks per second)
    public double getEncoderVelocity() {
        return shooter1.getVelocity();
    }

    public double getShooterRPM() {
        // Get velocity in ticks per second from shooter1 encoder and convert to RPM
        double ticksPerSecond = shooter1.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    // Shooter wheel velocity in meters per second
    public double getShooterMPS() {
        double rpm = getShooterRPM();
        double revsPerSecond = rpm / 60.0;
        double circumference = Math.PI * WHEEL_DIAMETER_METERS;
        return revsPerSecond * circumference;
    }

    // Get the target RPM
    public double getTargetRPM() {
        return targetRPM;
    }

    // Check if shooter is at target RPM within tolerance
    public boolean isAtTargetRPM() {
        return Math.abs(getShooterRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    private void setTargetRPM(double rpm) {
        targetRPM = rpm;

        if (rpm <= 0) {
            // Stop both motors
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterMode = ShooterMode.STOP;
        } else {
            // Only shooter1 uses velocity control (has encoder)
            double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
            shooter1.setVelocity(ticksPerSecond);

            // Shooter2 runs at equivalent power (no encoder feedback)
            double equivalentPower = rpm / SHOOTER_FULL_RPM;  // Scale to 0-1 range
            equivalentPower = Math.max(0, Math.min(1, equivalentPower)); // Clamp to valid range
            shooter2.setPower(-equivalentPower);  // Opposite direction

            shooterMode = ShooterMode.VELOCITY_CONTROL;
        }
    }

    public void shooterFull() {
        setTargetRPM(SHOOTER_FULL_RPM);
    }

    public void shooterHalf() {
        setTargetRPM(SHOOTER_HALF_RPM);
    }

    public void shooterQuarter() {
        setTargetRPM(SHOOTER_QUARTER_RPM);
    }

    public void shooterStop() {
        setTargetRPM(SHOOTER_STOP_RPM);
    }

    // Set custom RPM target
    public void setShooterRPM(double rpm) {
        setTargetRPM(rpm);
    }

    // Legacy method - now sets equivalent RPM
    public void setShooterPower(double power) {
        double rpm = power * SHOOTER_FULL_RPM;
        setTargetRPM(rpm);
    }

    public double getShooterPower() {
        return shooter1.getPower();
    }

    public double getShooter2Power() {
        return shooter2.getPower();
    }

    public boolean isDoingShooter() {
        return shooterMode == ShooterMode.VELOCITY_CONTROL;
    }

    // Get current RPM error for debugging
    public double getRPMError() {
        return targetRPM - getShooterRPM();
    }

    // Add telemetry data to the telemetry object
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter Status", shooterMode);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", getShooterRPM());
        telemetry.addData("RPM Error", "%.0f", getRPMError());
        telemetry.addData("At Target", isAtTargetRPM() ? "YES" : "NO");
        telemetry.addData("Shooter1 Power", "%.3f", getShooterPower());
        telemetry.addData("Shooter2 Power", "%.3f", getShooter2Power());
        telemetry.addData("Encoder Position", getEncoderPosition());
        telemetry.addData("Encoder Velocity", "%.1f tps", getEncoderVelocity());
        telemetry.addData("Wheel Speed", "%.2f m/s", getShooterMPS());
    }

    // Get telemetry data as string (alternative method)
    @SuppressLint("DefaultLocale")
    public String getTelemetryData() {
        return String.format("Target: %.0f RPM | Current: %.0f RPM | Error: %.0f | At Target: %s",
                targetRPM, getShooterRPM(), getRPMError(), isAtTargetRPM() ? "YES" : "NO");
    }
}