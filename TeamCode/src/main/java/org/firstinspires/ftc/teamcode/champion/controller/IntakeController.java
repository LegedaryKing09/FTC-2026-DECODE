package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class IntakeController {

    public static String INTAKE_NAME = "intake";
    public static double INTAKE_FULL_POWER = 1;
    public static double INTAKE_EJECT_POWER = -1;
    public static double INTAKE_STOP_POWER = 0;
    public static double INTAKE_HALF_POWER = 0.5;
    public static double INTAKE_QUARTER_POWER = 0.25;


    private enum IntakeMode {
        INTAKE, EJECT, STOP
    }

    private final DcMotor intake;
    private IntakeMode intakeMode = IntakeMode.STOP;

    public IntakeController(LinearOpMode opMode) {
        intake = opMode.hardwareMap.get(DcMotor.class, INTAKE_NAME);
    }

    public void intakeRest() {
        intake.setPower(INTAKE_STOP_POWER);
        intakeMode = IntakeMode.STOP;
    }

    public void intakeFull() {
        intake.setPower(INTAKE_FULL_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeHalf() {
        intake.setPower(INTAKE_HALF_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeQuarter() {
        intake.setPower(INTAKE_QUARTER_POWER);
        intakeMode = IntakeMode.INTAKE;
    }

    public void intakeEject() {
        intake.setPower(INTAKE_EJECT_POWER);
        intakeMode = IntakeMode.EJECT;
    }

    public double getIntakePower() {
        return intake.getPower();
    }

    public boolean isDoingIntake() {
        return intakeMode == IntakeMode.INTAKE;
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
        intakeMode = IntakeMode.INTAKE;
    }

}
