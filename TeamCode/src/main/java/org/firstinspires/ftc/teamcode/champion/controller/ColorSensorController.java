package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Objects;

public class ColorSensorController {
    public static String SENSOR_NAME = "colorSensor";
    private NormalizedColorSensor colorSensor;
    private volatile NormalizedRGBA colors;

    /**
     * MUST USE!!! IT IS IMPORTANT INITIALIZATION!!!
     */
    public ColorSensorController(LinearOpMode opMode) {
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, SENSOR_NAME);
    }
    /**
     * @param color Only accepts red, blue, and all else returns green. Not case-sensitive.
     * @return Color value
     */
    public float ColorValue(String color) {
        colors = colorSensor.getNormalizedColors();
        if(Objects.equals(color.toLowerCase(), "red")) {
            return colors.red;
        } else if(Objects.equals(color.toLowerCase(), "blue")) {
            return colors.blue;
        } else {
            return colors.green;
        }
    }
    public boolean IsGreen() {
        colors = colorSensor.getNormalizedColors();
        if(colors.green + colors.blue + colors.red > 0.008) {
            return (colors.green > colors.blue && colors.green > colors.red);
        } else {
            return false;
        }
    }
    public boolean IsPurple() {
        colors = colorSensor.getNormalizedColors();
        if(colors.green + colors.blue + colors.red > 0.008) {
            return (colors.blue > colors.green && colors.green / (colors.red + colors.blue + colors.green) < 0.31);
        } else {
            return false;
        }
    }
}