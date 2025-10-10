package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Pinpoint Diagnostic", group = "Testing")
public class PinpointDiagnostic extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime runtime = new ElapsedTime();
    private int consecutiveGoodReads = 0;
    private int consecutiveBadReads = 0;
    private int totalGoodReads = 0;
    private int totalBadReads = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Pinpoint...");
        telemetry.update();

        try {
            // Initialize Pinpoint directly (bypassing the drive controller for now)
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            telemetry.addData("Status", "Pinpoint hardware mapped successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize Pinpoint: " + e.getMessage());
            telemetry.addData("", "Check hardware configuration name 'odo'");
            telemetry.update();
            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        // Basic device checks
        telemetry.addData("Status", "Running basic device checks...");
        telemetry.update();

        // Check device ID and version
        int deviceId = pinpoint.getDeviceID();
        int deviceVersion = pinpoint.getDeviceVersion();

        telemetry.addData("Device ID", deviceId + (deviceId == 1 ? " (OK)" : " (WRONG - should be 1)"));
        telemetry.addData("Device Version", "0x" + Integer.toHexString(deviceVersion));

        if (deviceId != 1) {
            telemetry.addData("ERROR", "Device ID incorrect - check I2C connection");
            telemetry.update();
            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        // Configure with minimal settings first
        telemetry.addData("Status", "Configuring Pinpoint with basic settings...");
        telemetry.update();

        try {
            // Set encoder directions
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Set encoder resolution for goBILDA swingarm pods
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

            // Set conservative offsets (you'll need to adjust these for your robot)
            pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);

            // Reset and calibrate
            pinpoint.resetPosAndIMU();

            telemetry.addData("Status", "Configuration complete, waiting for calibration...");
            telemetry.update();

            // Wait for calibration to complete
            sleep(2000);

        } catch (Exception e) {
            telemetry.addData("ERROR", "Configuration failed: " + e.getMessage());
            telemetry.update();
            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        telemetry.addData("Status", "Ready! Press PLAY to start diagnostic");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.clear();

        while (opModeIsActive()) {
            // Get status before attempting to read data
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

            try {
                // Attempt to update data
                pinpoint.update();

                // Check if we got a bad read
                if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
                    consecutiveBadReads++;
                    totalBadReads++;
                    consecutiveGoodReads = 0;
                } else {
                    consecutiveGoodReads++;
                    totalGoodReads++;
                    consecutiveBadReads = 0;
                }

            } catch (Exception e) {
                telemetry.addData("EXCEPTION", "Update failed: " + e.getMessage());
                consecutiveBadReads++;
                totalBadReads++;
                consecutiveGoodReads = 0;
            }

            // Display comprehensive diagnostic information
            displayDiagnosticTelemetry(status);

            // Small delay
            sleep(50);
        }
    }

    private void displayDiagnosticTelemetry(GoBildaPinpointDriver.DeviceStatus status) {
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addLine();

        // Device status and health
        String statusColor = getStatusColor(status);
        telemetry.addData("Device Status", status.toString() + " " + statusColor);
        telemetry.addData("Device ID", pinpoint.getDeviceID());
        telemetry.addData("Device Version", "0x" + Integer.toHexString(pinpoint.getDeviceVersion()));
        telemetry.addLine();

        // I2C Communication Health
        int totalReads = totalGoodReads + totalBadReads;
        double successRate = totalReads > 0 ? (double)totalGoodReads / totalReads * 100.0 : 0.0;

        telemetry.addData("I2C Success Rate", "%.1f%% (%d/%d)", successRate, totalGoodReads, totalReads);
        telemetry.addData("Consecutive Good", consecutiveGoodReads);
        telemetry.addData("Consecutive Bad", consecutiveBadReads);
        telemetry.addLine();

        // Loop timing
        int loopTime = pinpoint.getLoopTime();
        double frequency = pinpoint.getFrequency();
        String timingHealth = getTimingHealth(loopTime, frequency);

        telemetry.addData("Loop Time", "%d μs %s", loopTime, timingHealth);
        telemetry.addData("Frequency", "%.1f Hz %s", frequency, timingHealth);
        telemetry.addLine();

        // Encoder status
        int xEncoder = pinpoint.getEncoderX();
        int yEncoder = pinpoint.getEncoderY();

        telemetry.addData("X Encoder", "%d ticks %s", xEncoder, xEncoder == 0 ? "(No movement or not connected?)" : "");
        telemetry.addData("Y Encoder", "%d ticks %s", yEncoder, yEncoder == 0 ? "(No movement or not connected?)" : "");
        telemetry.addLine();

        // Position data (only if not in bad read state)
        if (status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            telemetry.addData("Position", "X: %.1f mm, Y: %.1f mm",
                    pinpoint.getPosX(DistanceUnit.MM),
                    pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading", "%.1f°",
                    pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocities", "X: %.1f mm/s, Y: %.1f mm/s, H: %.1f°/s",
                    pinpoint.getVelX(DistanceUnit.MM),
                    pinpoint.getVelY(DistanceUnit.MM),
                    Math.toDegrees(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
        } else {
            telemetry.addData("Position Data", "UNAVAILABLE (Bad Read)");
        }
        telemetry.addLine();

        // Troubleshooting recommendations
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            telemetry.addLine("TROUBLESHOOTING RECOMMENDATIONS:");
            telemetry.addLine("1. Check I2C wiring (SDA, SCL, 3.3V, GND)");
            telemetry.addLine("2. Verify odometry pod connections");
            telemetry.addLine("3. Check for I2C address conflicts");
            telemetry.addLine("4. Ensure stable power supply");

            if (consecutiveBadReads > 10) {
                telemetry.addLine("CRITICAL: Too many consecutive bad reads!");
                telemetry.addLine("Hardware issue likely present.");
            }
        } else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED) {
            telemetry.addLine("ERROR: No odometry pods detected!");
            telemetry.addLine("Check encoder cable connections to Pinpoint");
        } else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED) {
            telemetry.addLine("ERROR: X (forward) odometry pod not detected!");
            telemetry.addLine("Check X encoder cable connection");
        } else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            telemetry.addLine("ERROR: Y (strafe) odometry pod not detected!");
            telemetry.addLine("Check Y encoder cable connection");
        } else if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
            telemetry.addLine("Device is calibrating IMU - keep robot stationary");
        } else if (loopTime > 1100 || frequency < 900) {
            telemetry.addLine("WARNING: Poor timing performance detected");
            telemetry.addLine("This may indicate hardware issues");
        }

        telemetry.update();
    }

    private String getStatusColor(GoBildaPinpointDriver.DeviceStatus status) {
        switch (status) {
            case READY:
                return "✓ GOOD";
            case CALIBRATING:
                return "⏳ WAIT";
            case FAULT_BAD_READ:
                return "✗ I2C ERROR";
            case FAULT_NO_PODS_DETECTED:
                return "✗ NO PODS";
            case FAULT_X_POD_NOT_DETECTED:
                return "✗ NO X POD";
            case FAULT_Y_POD_NOT_DETECTED:
                return "✗ NO Y POD";
            case FAULT_IMU_RUNAWAY:
                return "✗ IMU ERROR";
            case NOT_READY:
                return "⏳ STARTING";
            default:
                return "? UNKNOWN";
        }
    }

    private String getTimingHealth(int loopTime, double frequency) {
        if (loopTime < 500 || loopTime > 1100 || frequency < 900 || frequency > 2000) {
            return "⚠ POOR";
        } else {
            return "✓ GOOD";
        }
    }
}