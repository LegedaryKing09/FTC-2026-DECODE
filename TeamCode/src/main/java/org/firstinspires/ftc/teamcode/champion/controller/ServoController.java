package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ServoController {

    public static String AXON_MINI_NAME = "spin_indexer";
    public static double AXON_MINI_MIN_POSITION = 0.0;
    public static double AXON_MINI_MAX_POSITION = 1.0;
    public static double AXON_MINI_HALF_POSITION = 0.5;
    public static double AXON_MINI_QUARTER_POSITION = 0.25;
    public static double AXON_MINI_THREE_QUARTER_POSITION = 0.75;

    private final Servo axonMini;

    public ServoController(LinearOpMode opMode) {
        axonMini = opMode.hardwareMap.get(Servo.class, AXON_MINI_NAME);
        // Uncomment the line below if you need to reverse the servo direction
        // axonMini.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double position) {
        axonMini.setPosition(position);
    }

    public double getPosition() {
        return axonMini.getPosition();
    }

    public void setToMin() {
        setPosition(AXON_MINI_MIN_POSITION);
    }

    public void setToMax() {
        setPosition(AXON_MINI_MAX_POSITION);
    }

    public void setToHalf() {
        setPosition(AXON_MINI_HALF_POSITION);
    }

    public void setToQuarter() {
        setPosition(AXON_MINI_QUARTER_POSITION);
    }

    public void setToThreeQuarter() {
        setPosition(AXON_MINI_THREE_QUARTER_POSITION);
    }
}