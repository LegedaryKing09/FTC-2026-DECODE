package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class IntakeController {

    public static String ROLLER_NAME = "roller";
    public static double ROLLER_FULL_POWER = 1;
    public static double ROLLER_EJECT_POWER = -1;
    public static double ROLLER_STOP_POWER = 0;
    public static double ROLLER_HALF_POWER = 0.5;
    public static double ROLLER_QUARTER_POWER = 0.25;


    private enum IntakeMode {
        INTAKE, EJECT, STOP
    }

    private DcMotor roller;
    private IntakeMode intakeMode = IntakeMode.STOP;

    public IntakeController(LinearOpMode opMode) {
        roller = opMode.hardwareMap.get(DcMotor.class, ROLLER_NAME);
    }

    public void intakeRest() {
        roller.setPower(ROLLER_STOP_POWER);
        intakeMode = IntakeMode.STOP;
    }

    public void intakeFull() {
        roller.setPower(ROLLER_FULL_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeHalf() {
        roller.setPower(ROLLER_HALF_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeQuarter() {
        roller.setPower(ROLLER_QUARTER_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeEject() {
        roller.setPower(ROLLER_EJECT_POWER);
        intakeMode = IntakeMode.EJECT;
    }

    public double getRollerPower() {
        return roller.getPower();
    }

    public boolean isDoingIntake() {
        return intakeMode == IntakeMode.INTAKE;
    }

    public void setRollerPower(double power) {
        roller.setPower(power);
        intakeMode = IntakeMode.INTAKE;
    }

}
