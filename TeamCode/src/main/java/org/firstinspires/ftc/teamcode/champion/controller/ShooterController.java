package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterController {
    public static String SHOOTER_NAME = "shooter";
    public static double SHOOTER_FULL_POWER = 1;
    public static double SHOOTER_HALF_POWER = 0.5;
    public static double SHOOTER_QUARTER_POWER = 0.25;
    public static double SHOOTER_STOP_POWER = 0;

    private enum ShooterMode {
        SHOOT, STOP
    }

    private DcMotor shooter;
    private ShooterMode shooterMode = ShooterMode.STOP;

    public ShooterController(LinearOpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotor.class, SHOOTER_NAME);
    }

    public void shooterFull() {
        shooter.setPower(SHOOTER_FULL_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterHalf() {
        shooter.setPower(SHOOTER_HALF_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterQuarter() {
        shooter.setPower(SHOOTER_QUARTER_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void shooterStop() {
        shooter.setPower(SHOOTER_STOP_POWER);
        shooterMode = ShooterMode.SHOOT;
    }

    public void setShooterPower(double power) {
        shooter.setPower(power);
        shooterMode = ShooterMode.SHOOT;
    }

    public boolean isDoingShooter() {
        return shooterMode == ShooterMode.SHOOT;
    }
}
