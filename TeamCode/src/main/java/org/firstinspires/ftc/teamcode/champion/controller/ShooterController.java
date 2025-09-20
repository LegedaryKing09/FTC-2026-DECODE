package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ShooterController {
    public static String SHOOTER_NAME1 = "shooter1";
    public static String SHOOTER_NAME2 = "shooter2";
    public static double SHOOTER_FULL_POWER = 1;
    public static double SHOOTER_HALF_POWER = 0.5;
    public static double SHOOTER_QUARTER_POWER = 0.25;
    public static double SHOOTER_STOP_POWER = 0;

    private enum ShooterMode {
        SHOOT, STOP
    }

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private ShooterMode shooterMode = ShooterMode.STOP;

    public ShooterController(LinearOpMode opMode) {
        shooter1 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME1);
        shooter2 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getShooterRPM() {
        // velocity in rpm
        double ticksPerSecond = shooter1.getVelocity();
        double TICKS_PER_REV = 28;
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    //shooter wheel in mps
    public double getShooterMPS() {
        double rpm = getShooterRPM();
        double revsPerSecond = rpm / 60.0;
        double WHEEL_DIAMETER_METERS = 0.10795;
        double circumference = Math.PI * WHEEL_DIAMETER_METERS;
        return revsPerSecond * circumference;
    }

    public void shooterFull() {
        shooter1.setPower(SHOOTER_FULL_POWER);
        shooter2.setPower(-SHOOTER_FULL_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterHalf() {
        shooter1.setPower(SHOOTER_HALF_POWER);
        shooter2.setPower(-SHOOTER_HALF_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterQuarter() {
        shooter1.setPower(SHOOTER_QUARTER_POWER);
        shooter2.setPower(-SHOOTER_QUARTER_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterStop() {
        shooter1.setPower(SHOOTER_STOP_POWER);
        shooter2.setPower(-SHOOTER_STOP_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(-power);
        shooterMode = ShooterMode.SHOOT;
    }

    public double getShooterPower() {
        return shooter1.getPower();
    }

    public boolean isDoingShooter() {
        return shooterMode == ShooterMode.SHOOT;
    }
}
