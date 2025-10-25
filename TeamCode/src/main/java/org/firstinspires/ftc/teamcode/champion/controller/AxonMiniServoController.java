package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class AxonMiniServoController {

    public static String AXON_MINI_NAME = "ramp";
    public static String AXON_MINI_SECOND_NAME = "ramp2"; // Second servo for opposite direction
    public static double AXON_MINI_MIN_POSITION = 0.0;
    public static double AXON_MINI_MAX_POSITION = 1.0;
    public static double AXON_MINI_HALF_POSITION = 0.5;
    public static double AXON_MINI_QUARTER_POSITION = 0.25;
    public static double AXON_MINI_THREE_QUARTER_POSITION = 0.75;

    private enum ServoMode {
        POSITION
    }

    private final Servo axonMini;
    private Servo axonMiniSecond; // Second servo, null if not found
    private final ServoMode servoMode = ServoMode.POSITION;

    public AxonMiniServoController(LinearOpMode opMode) {
        axonMini = opMode.hardwareMap.get(Servo.class, AXON_MINI_NAME);
        // Uncomment the line below if you need to reverse the servo direction
        axonMini.setDirection(Servo.Direction.FORWARD);

        // Try to get second servo, set to null if not found
        try {
            axonMiniSecond = opMode.hardwareMap.get(Servo.class, AXON_MINI_SECOND_NAME);
            axonMiniSecond.setDirection(Servo.Direction.REVERSE); // Opposite direction for second servo
        } catch (Exception e) {
            axonMiniSecond = null; // Second servo not configured
        }
    }

    public void setPosition(double position) {
        axonMini.setPosition(position);
        if (axonMiniSecond != null) {
            axonMiniSecond.setPosition(1.0 - position); // Opposite position for second servo
        }
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

    public void stop() {
        setPosition(AXON_MINI_HALF_POSITION);
    }

    public void forwardFull() {
        setPosition(AXON_MINI_MAX_POSITION);
    }

    public void forwardHalf() {
        setPosition(0.75);
    }

    public void forwardQuarter() {
        setPosition(0.625);
    }

    public void forwardSlow() {
        setPosition(0.55);
    }

    public void reverseFull() {
        setPosition(AXON_MINI_MIN_POSITION);
    }

    public void reverseHalf() {
        setPosition(0.25);
    }

    public void reverseQuarter() {
        setPosition(0.375);
    }

    public void reverseSlow() {
        setPosition(0.45);
    }

    public double getPower() {
        // Map position back to power (-1 to 1)
        return (getPosition() * 2.0) - 1.0;
    }

    public boolean isMovingForward() {
        return getPosition() > AXON_MINI_HALF_POSITION;
    }

    public boolean isMovingReverse() {
        return getPosition() < AXON_MINI_HALF_POSITION;
    }

    public boolean isStopped() {
        return getPosition() == AXON_MINI_HALF_POSITION;
    }

    public void setPower(double power) {
        // Map power (-1 to 1) to position (0 to 1)
        double position = (power + 1.0) / 2.0;
        setPosition(position);
    }

    public ServoMode getMode() {
        return servoMode;
    }

    public Servo getServo() {
        return axonMini;
    }

    public Servo getSecondServo() {
        return axonMiniSecond;
    }

    public boolean hasSecondServo() {
        return axonMiniSecond != null;
    }
}