package org.firstinspires.ftc.teamcode.champion.controller;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LoaderController {
    public static String LOADER_NAME = "loader";
    public static double LOADER_FULL_POWER = 1;
    public static double LOADER_HALF_POWER = 0.5;
    public static double LOADER_QUARTER_POWER = 0.25;
    public static double LOADER_STOP_POWER = 0;

    private enum LoaderMode {
        TRANSFER, STOP
    }

    private DcMotor loader;
    private LoaderMode loaderMode = LoaderMode.STOP;

    public LoaderController(LinearOpMode opMode) {
        loader = opMode.hardwareMap.get(DcMotor.class, LOADER_NAME);
    }

    public void loaderFull() {
        loader.setPower(LOADER_FULL_POWER);
        loaderMode = LoaderMode.TRANSFER;
    }

    public void loaderHalf() {
        loader.setPower(LOADER_HALF_POWER);
        loaderMode = LoaderMode.TRANSFER;
    }

    public void loaderQuarter() {
        loader.setPower(LOADER_QUARTER_POWER);
        loaderMode = LoaderMode.TRANSFER;
    }

    public void loaderStop() {
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setPower(LOADER_STOP_POWER);
        loaderMode = LoaderMode.TRANSFER;
    }

    public double getLoaderPower() {
        return loader.getPower();
    }

    public void setLoaderPower(double power) {
        loader.setPower(power);
        loaderMode = LoaderMode.TRANSFER;
    }

    public boolean isDoingLoader() {
        return loaderMode == LoaderMode.TRANSFER;
    }
}

