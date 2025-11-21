package org.firstinspires.ftc.teamcode.champion.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class AxonAnalog extends LinearOpMode {

    Servo rightAxon, leftAxon;
    AnalogInput rightAnalog, leftAnalog;

    public static double RIGHTVOLTS = 0;
    public static double LEFTVOLTS = 0;
    public static double SERVOPOSITION1 = 0;
    public static double SERVOPOSITION2 = 0;

    @Override
    public void runOpMode() {


        rightAxon = hardwareMap.get(Servo.class, "rightAxon");
//        leftAxon = hardwareMap.get(Servo.class, "leftAxon");

        rightAnalog = hardwareMap.get(AnalogInput.class, "rightAnalog");
//        leftAnalog = hardwareMap.get(AnalogInput.class, "leftAnalog");

        telemetry.addData("Right Volts: ", rightAnalog.getVoltage());
//        telemetry.addData("Left Volts: ", leftAnalog.getVoltage());

        /**
         * 3.3 is the battery Voltage
         */
        LEFTVOLTS = (-(leftAnalog.getVoltage() - 1.958)) / 3.3;//volts at 0
//        RIGHTVOLTS = (rightAnalog.getVoltage() - 2.92) / 3.3;//Subtracted values are offsets
        waitForStart();

        while (opModeIsActive()) {
            rightAxon.setPosition(SERVOPOSITION1 - RIGHTVOLTS + rightAxon.getPosition());
//            leftAxon.setPosition(SERVOPOSITION2 - LEFTVOLTS + leftAxon.getPosition());
        }
    }
}