package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceController {
    public static final String SENSOR_NAME = "distance";
    private final DistanceSensor distanceSensor;
    private final Rev2mDistanceSensor sensorTimeOfFlight;

    public DistanceController(HardwareMap hardwareMap) {
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, SENSOR_NAME);
        this.sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;
    }

    /**
     * Get distance measurement in specified unit
     * @param unit The distance unit to return
     * @return Distance measurement, or -1 if sensor is not available
     */
    public double getDistance(DistanceUnit unit) {
        if (distanceSensor != null) {
            return distanceSensor.getDistance(unit);
        }
        return -1.0; // Error value
    }

    /**
     * Get distance in centimeters (convenience method)
     */
    public double getDistanceCM() {
        return getDistance(DistanceUnit.CM);
    }

    /**
     * Get distance in inches (convenience method)
     */
    public double getDistanceInches() {
        return getDistance(DistanceUnit.INCH);
    }

    /**
     * Get distance in millimeters (convenience method)
     */
    public double getDistanceMM() {
        return getDistance(DistanceUnit.MM);
    }

    /**
     * Check if an object is within a specified range
     * @param maxDistance Maximum distance threshold
     * @param unit Unit of measurement
     * @return true if object is within range, false otherwise
     */
    public boolean isObjectInRange(double maxDistance, DistanceUnit unit) {
        double currentDistance = getDistance(unit);
        return currentDistance > 0 && currentDistance <= maxDistance;
    }

    /**
     * Check if sensor is properly initialized and working
     */
    public boolean isSensorAvailable() {
        return distanceSensor != null;
    }

    /**
     * Get sensor name for debugging
     */
    public String getSensorName() {
        return SENSOR_NAME;
    }

    /**
     * Get the Rev2m specific sensor if needed for advanced features
     */
    public Rev2mDistanceSensor getTimeOfFlightSensor() {
        return sensorTimeOfFlight;
    }
}