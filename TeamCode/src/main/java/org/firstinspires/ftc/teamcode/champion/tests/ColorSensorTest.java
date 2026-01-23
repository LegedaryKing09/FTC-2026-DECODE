package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.ColorSensorController;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the color sensor controller
        ColorSensorController colorController = new ColorSensorController(this);

        telemetry.addData("Status", "Color sensor test ready");
        telemetry.addData("Controls", "A: Snapshot, X: Exit");
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
            } else if (isPurple) {
                dominantColor = "PURPLE";
            } else {
                dominantColor = getDominantColor(redValue, greenValue, blueValue);
            }

            telemetry.addData("Red", "%.3f", redValue);
            telemetry.addData("Green", "%.3f", greenValue);
            telemetry.addData("Blue", "%.3f", blueValue);
            telemetry.addData("Intensity", "%.3f", totalIntensity);

            if (totalIntensity > 0) {
                telemetry.addData("Red %", "%.1f%%", (redValue / totalIntensity) * 100);
                telemetry.addData("Green %", "%.1f%%", (greenValue / totalIntensity) * 100);
                telemetry.addData("Blue %", "%.1f%%", (blueValue / totalIntensity) * 100);
            }

            telemetry.addData("Dominant", dominantColor);
            telemetry.addData("Green Detected", isGreen);
            telemetry.addData("Purple Detected", isPurple);
            telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
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
        telemetry.addData("SNAPSHOT", "Time: %.1f sec", runtime.seconds());
        telemetry.addData("Values", "R:%.3f G:%.3f B:%.3f", red, green, blue);
        telemetry.addData("Green", isGreen);
        telemetry.addData("Purple", isPurple);

        float max = Math.max(Math.max(red, green), blue);
        float min = Math.min(Math.min(red, green), blue);
        float range = max - min;
        telemetry.addData("Range", "%.3f (Sat: %.1f%%)",
                range, max > 0 ? (range / max) * 100 : 0);
    }
}