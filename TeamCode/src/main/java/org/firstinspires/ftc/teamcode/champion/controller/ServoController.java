package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
public class ServoController {

    public static String AXON_MINI_NAME = "spin_indexer";
    public static double AXON_MINI_FULL_POWER = 1.0;
    public static double AXON_MINI_REVERSE_POWER = -1.0;
    public static double AXON_MINI_STOP_POWER = 0.0;
    public static double AXON_MINI_HALF_POWER = 0.5;
    public static double AXON_MINI_QUARTER_POWER = 0.25;
    public static double AXON_MINI_SLOW_POWER = 0.1;

    private enum AxonMiniMode {
        FORWARD, REVERSE, STOP
    }

    private final CRServo axonMini;
    private AxonMiniMode axonMiniMode = AxonMiniMode.STOP;

    public ServoController(LinearOpMode opMode) {
        axonMini = opMode.hardwareMap.get(CRServo.class, AXON_MINI_NAME);
        // Uncomment the line below if you need to reverse the servo direction
        // axonMini.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
        axonMini.setPower(AXON_MINI_STOP_POWER);
        axonMiniMode = AxonMiniMode.STOP;
    }

    public void forwardFull() {
        axonMini.setPower(AXON_MINI_FULL_POWER);
        axonMiniMode = AxonMiniMode.FORWARD;
    }

    public void forwardHalf() {
        axonMini.setPower(AXON_MINI_HALF_POWER);
        axonMiniMode = AxonMiniMode.FORWARD;
    }

    public void forwardQuarter() {
        axonMini.setPower(AXON_MINI_QUARTER_POWER);
        axonMiniMode = AxonMiniMode.FORWARD;
    }

    public void forwardSlow() {
        axonMini.setPower(AXON_MINI_SLOW_POWER);
        axonMiniMode = AxonMiniMode.FORWARD;
    }

    public void reverseFull() {
        axonMini.setPower(AXON_MINI_REVERSE_POWER);
        axonMiniMode = AxonMiniMode.REVERSE;
    }

    public void reverseHalf() {
        axonMini.setPower(-AXON_MINI_HALF_POWER);
        axonMiniMode = AxonMiniMode.REVERSE;
    }

    public void reverseQuarter() {
        axonMini.setPower(-AXON_MINI_QUARTER_POWER);
        axonMiniMode = AxonMiniMode.REVERSE;
    }

    public void reverseSlow() {
        axonMini.setPower(-AXON_MINI_SLOW_POWER);
        axonMiniMode = AxonMiniMode.REVERSE;
    }

    public double getPower() {
        return axonMini.getPower();
    }

    public boolean isMovingForward() {
        return axonMiniMode == AxonMiniMode.FORWARD;
    }

    public boolean isMovingReverse() {
        return axonMiniMode == AxonMiniMode.REVERSE;
    }

    public boolean isStopped() {
        return axonMiniMode == AxonMiniMode.STOP;
    }

    public void setPower(double power) {
        axonMini.setPower(power);
        if (power > 0) {
            axonMiniMode = AxonMiniMode.FORWARD;
        } else if (power < 0) {
            axonMiniMode = AxonMiniMode.REVERSE;
        } else {
            axonMiniMode = AxonMiniMode.STOP;
        }
    }

    public AxonMiniMode getMode() {
        return axonMiniMode;
    }
}