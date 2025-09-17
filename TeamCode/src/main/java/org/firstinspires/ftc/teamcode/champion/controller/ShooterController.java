package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    private final DcMotor shooter1;
    private final DcMotor shooter2;
    private ShooterMode shooterMode = ShooterMode.STOP;

    public ShooterController(LinearOpMode opMode) {
        shooter1 = opMode.hardwareMap.get(DcMotor.class, SHOOTER_NAME1);
        shooter2 = opMode.hardwareMap.get(DcMotor.class, SHOOTER_NAME2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
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
