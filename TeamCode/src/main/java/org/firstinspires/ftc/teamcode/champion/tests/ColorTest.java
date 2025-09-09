package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.ColorSensorController;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorTest extends LinearOpMode {

    private ColorSensorController colorController;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the color sensor controller
        colorController = new ColorSensorController(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized - Ready to test color sensor");
        telemetry.addData("Instructions", "Press PLAY to start color sensor testing");
        telemetry.addData("Controls", "Press A to take snapshot, X to exit");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get individual color values using the controller methods
            float redValue = colorController.ColorValue("red");
            float greenValue = colorController.ColorValue("green");
            float blueValue = colorController.ColorValue("blue");

            // Get color detection results
            boolean isGreen = colorController.IsGreen();
            boolean isPurple = colorController.IsPurple();

            // Calculate additional useful metrics
            float totalIntensity = redValue + greenValue + blueValue;
            String dominantColor;
            if(totalIntensity <= 0.008) {
                dominantColor = "None";
            } else {
                dominantColor = getDominantColor(redValue, greenValue, blueValue);
            }

            // Display basic color values
            telemetry.addData("=== RAW COLOR VALUES ===", "");
            telemetry.addData("Red", "%.3f", redValue);
            telemetry.addData("Green", "%.3f", greenValue);
            telemetry.addData("Blue", "%.3f", blueValue);
            telemetry.addData("Total Intensity", "%.3f", totalIntensity);

            // Display color percentages
            telemetry.addData("=== COLOR PERCENTAGES ===", "");
            if (totalIntensity > 0) {
                telemetry.addData("Red %", "%.1f%%", (redValue / totalIntensity) * 100);
                telemetry.addData("Green %", "%.1f%%", (greenValue / totalIntensity) * 100);
                telemetry.addData("Blue %", "%.1f%%", (blueValue / totalIntensity) * 100);
            }

            // Display color detection results
            telemetry.addData("=== COLOR DETECTION ===", "");
            telemetry.addData("Dominant Color", dominantColor);
            telemetry.addData("Is Green?", isGreen ? "YES" : "NO");
            telemetry.addData("Is Purple?", isPurple ? "YES" : "NO");

            // Display color ratios for analysis
            telemetry.addData("=== COLOR RATIOS ===", "");
            if (redValue > 0) {
                telemetry.addData("Green/Red Ratio", "%.2f", greenValue / redValue);
                telemetry.addData("Blue/Red Ratio", "%.2f", blueValue / redValue);
            }
            if (greenValue > 0) {
                telemetry.addData("Blue/Green Ratio", "%.2f", blueValue / greenValue);
            }

            // Display runtime and controls
            telemetry.addData("=== STATUS ===", "");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.addData("Controls", "A: Snapshot, X: Exit");

            // Take snapshot if A button is pressed
            if (gamepad1.a) {
                takeSnapshot(redValue, greenValue, blueValue, isGreen, isPurple);
                // Small delay to prevent multiple snapshots
                sleep(200);
            }

            // Exit if X button is pressed
            if (gamepad1.x) {
                telemetry.addData("Status", "Exiting color sensor test...");
                telemetry.update();
                break;
            }

            telemetry.update();

            // Small delay to make telemetry readable
            sleep(50);
        }
    }

    /**
     * Determines which color has the highest value
     */
    private String getDominantColor(float red, float green, float blue) {
        if (red >= green && red >= blue) {
            return "RED";
        } else if (green >= red && green >= blue) {
            return "GREEN";
        } else {
            return "BLUE";
        }
    }

    /**
     * Takes a snapshot of current color readings for detailed analysis
     */
    private void takeSnapshot(float red, float green, float blue, boolean isGreen, boolean isPurple) {
        telemetry.addLine("=== SNAPSHOT TAKEN ===");
        telemetry.addData("Snapshot Time", "%.2f seconds", runtime.seconds());
        telemetry.addData("R/G/B Values", "%.3f / %.3f / %.3f", red, green, blue);
        telemetry.addData("Green Detected", isGreen);
        telemetry.addData("Purple Detected", isPurple);

        // Additional analysis
        float max = Math.max(Math.max(red, green), blue);
        float min = Math.min(Math.min(red, green), blue);
        float range = max - min;

        telemetry.addData("Max Value", "%.3f", max);
        telemetry.addData("Min Value", "%.3f", min);
        telemetry.addData("Range", "%.3f", range);
        telemetry.addData("Saturation", range > 0 ? "%.1f%%" : "0%",
                max > 0 ? (range / max) * 100 : 0);

        telemetry.addLine("=====================");
    }
}