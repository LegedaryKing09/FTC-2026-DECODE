package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.champion.controller.DistanceController;

@Autonomous(name = "Distance Sensor Test", group = "Test")
public class DistanceTest extends LinearOpMode {
    private DistanceController distanceController;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Distance Sensor Test");
        telemetry.update();
        // Initialize the distance controller
        try {
            distanceController = new DistanceController(hardwareMap);
            telemetry.addData("Sensor", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize sensor: " + e.getMessage());
            telemetry.update();
            return;
        }

        // Test sensor availability
        testSensorAvailability();
        telemetry.addData("Status", "Ready to start tests");
        telemetry.addData("Instructions", "Press play to begin distance sensor tests");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            runDistanceTests();
        }
    }

    private void testSensorAvailability() {
        telemetry.addData("Sensor Available", distanceController.isSensorAvailable());
        telemetry.addData("Sensor Name", distanceController.getSensorName());
        telemetry.update();
        sleep(1000);
    }

    private void runDistanceTests() {
        telemetry.clearAll();
        telemetry.addData("Status", "Running Distance Tests");
        telemetry.addLine("Move objects in front of sensor to test");
        telemetry.addLine("---");
        telemetry.update();

        while (opModeIsActive()) {
            // Test 1: Basic distance readings in different units
            testBasicDistanceReadings();

            // Test 2: Range detection tests
            testRangeDetection();

            // Test 3: Performance monitoring
            testPerformance();

            // Update telemetry
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();

            sleep(100); // Update at ~10Hz
        }
    }

    private void testBasicDistanceReadings() {
        // Test different units
        double distanceCM = distanceController.getDistanceCM();
        double distanceInches = distanceController.getDistanceInches();
        double distanceMM = distanceController.getDistanceMM();
        double distanceMeters = distanceController.getDistance(DistanceUnit.METER);

        telemetry.addData("Distance (CM)", "%.2f", distanceCM);
        telemetry.addData("Distance (Inches)", "%.2f", distanceInches);
        telemetry.addData("Distance (MM)", "%.2f", distanceMM);
        telemetry.addData("Distance (Meters)", "%.4f", distanceMeters);

        // Test for invalid readings
        if (distanceCM < 0) {
            telemetry.addData("WARNING", "Sensor returning invalid readings");
        }
    }

    private void testRangeDetection() {
        // Test various range thresholds
        boolean within10cm = distanceController.isObjectInRange(10.0, DistanceUnit.CM);
        boolean within20cm = distanceController.isObjectInRange(20.0, DistanceUnit.CM);
        boolean within5inches = distanceController.isObjectInRange(5.0, DistanceUnit.INCH);

        telemetry.addData("Within 10cm", within10cm ? "YES" : "NO");
        telemetry.addData("Within 20cm", within20cm ? "YES" : "NO");
        telemetry.addData("Within 5in", within5inches ? "YES" : "NO");

        // Visual indicator for close objects
        if (within10cm) {
            telemetry.addData("ALERT", "OBJECT VERY CLOSE!");
        }
    }

    private void testPerformance() {
        // Test reading speed/consistency
        long startTime = System.nanoTime();
        double distance = distanceController.getDistanceCM();
        long endTime = System.nanoTime();

        long readingTime = (endTime - startTime) / 1000; // Convert to microseconds

        telemetry.addData("Reading Time", "%d μs", readingTime);

        // Test for reasonable values (REV 2m sensor range is 5mm to 2000mm)
        if (distance > 0 && distance < 5) {
            telemetry.addData("WARNING", "Distance too close (< 5mm)");
        } else if (distance > 2000) {
            telemetry.addData("WARNING", "Distance beyond max range (> 2000mm)");
        } else if (distance > 0) {
            telemetry.addData("Status", "Reading within normal range");
        }
    }
}