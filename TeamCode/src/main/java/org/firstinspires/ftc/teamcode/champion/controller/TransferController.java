package org.firstinspires.ftc.teamcode.champion.controller;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class TransferController {
    public static String TRANSFER_NAME = "transfer";
    public static double TRANSFER_FULL_POWER = 1;
    public static double TRANSFER_EJECT_POWER = -1;
    public static double TRANSFER_HALF_POWER = 0.5;
    public static double TRANSFER_QUARTER_POWER = 0.25;
    public static double TRANSFER_STOP_POWER = 0;

    private enum TransferMode {
        TRANSFER, STOP, EJECT
    }

    private final DcMotor transfer;
    private TransferMode transferMode = TransferMode.STOP;

    public TransferController(LinearOpMode opMode) {
        transfer = opMode.hardwareMap.get(DcMotor.class, TRANSFER_NAME);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void transferFull() {
        transfer.setPower(TRANSFER_FULL_POWER);
        transferMode = TransferMode.TRANSFER;
    }

    public void transferEject() {
        transfer.setPower(TRANSFER_EJECT_POWER);
        transferMode = TransferMode.EJECT;
    }

    public void transferHalf() {
        transfer.setPower(TRANSFER_HALF_POWER);
        transferMode = TransferMode.TRANSFER;
    }

    public void transferQuarter() {
        transfer.setPower(TRANSFER_QUARTER_POWER);
        transferMode = TransferMode.TRANSFER;
    }

    public void transferStop() {
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(TRANSFER_STOP_POWER);
        transferMode = TransferMode.TRANSFER;
    }

    public double getTransferPower() {
        return transfer.getPower();
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
        transferMode = TransferMode.TRANSFER;
    }

    public boolean isDoingTransfer() {
        return transferMode == TransferMode.TRANSFER;
    }
}

