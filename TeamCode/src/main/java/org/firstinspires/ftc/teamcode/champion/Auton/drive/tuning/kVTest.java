package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;

@Autonomous(name = "Forward Ramp Logger (kV)", group = "Tuning")
public class kVTest extends LinearOpMode {

    // Tuning parameters
    private static final double MAX_POWER = 0.6; // Maximum power to ramp up to
    private static final double RAMP_DURATION = 8.0; // Duration of ramp in seconds
    private FileWriter csvWriter;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        // Set directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset and configure encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Get voltage sensor
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Open CSV file on RC storage - try multiple locations
        String[] possiblePaths = {
                "/sdcard/FIRST/data/FORWARD_RAMP.csv",
                "/sdcard/Download/FORWARD_RAMP.csv",
                "/sdcard/FORWARD_RAMP.csv"
        };

        boolean fileOpened = false;
        String successPath = "";

        for (String path : possiblePaths) {
            try {
                // Try to create parent directories if they don't exist
                java.io.File file = new java.io.File(path);
                java.io.File parentDir = file.getParentFile();
                if (parentDir != null && !parentDir.exists()) {
                    parentDir.mkdirs();
                }

                csvWriter = new FileWriter(path);
                csvWriter.write("time,power,voltageCommand,velocityInchesPerSec,leftVelocity,rightVelocity\n");
                csvWriter.flush(); // Force write immediately
                successPath = path;
                fileOpened = true;
                telemetry.addLine("✓ CSV file opened at: " + path);
                break;
            } catch (IOException e) {
                // Try next path
            }
        }

        if (!fileOpened) {
            telemetry.addLine("ERROR: Could not open CSV file at any location");
            telemetry.addLine("Tried paths:");
            for (String p : possiblePaths) {
                telemetry.addLine("  " + p);
            }
            telemetry.update();
        }

        telemetry.addLine("==================================");
        telemetry.addLine("FORWARD RAMP LOGGER (kV)");
        telemetry.addLine("==================================");
        telemetry.addLine();
        telemetry.addLine("This will log power vs velocity data");
        telemetry.addLine("to calculate kV (velocity gain).");
        telemetry.addLine();
        telemetry.addLine("Requirements:");
        telemetry.addLine("✓ kS already tuned");
        telemetry.addLine("✓ ~10 feet of clear space ahead");
        telemetry.addLine("✓ Robot placed at starting position");
        telemetry.addLine();
        telemetry.addData("Ramp Duration", "%.1f seconds", RAMP_DURATION);
        telemetry.addData("Max Power", "%.2f", MAX_POWER);
        telemetry.addLine();
        telemetry.addLine("After run, visit:");
        telemetry.addLine("http://192.168.43.1:8080/files");
        telemetry.addLine("Download: /sdcard/FORWARD_RAMP.csv");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            closeCSV();
            return;
        }

        telemetry.clearAll();
        telemetry.addLine("==================================");
        telemetry.addLine("RUNNING RAMP TEST");
        telemetry.addLine("==================================");
        telemetry.addLine("Do not touch the robot!");
        telemetry.update();

        double startTime = System.nanoTime() / 1e9;

        while (opModeIsActive()) {
            double currentTime = System.nanoTime() / 1e9;
            double elapsedTime = currentTime - startTime;

            // Check if ramp is complete
            if (elapsedTime > RAMP_DURATION) {
                break;
            }

            // Calculate ramped power (linear increase)
            double power = (elapsedTime / RAMP_DURATION) * MAX_POWER;

            // Apply power to motors
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            // Get velocities (ticks per second)
            double lfVel = leftFront.getVelocity();
            double lbVel = leftBack.getVelocity();
            double rfVel = rightFront.getVelocity();
            double rbVel = rightBack.getVelocity();

            // Calculate average velocity
            double leftVel = (lfVel + lbVel) / 2.0;
            double rightVel = (rfVel + rbVel) / 2.0;
            double avgVel = (leftVel + rightVel) / 2.0;

            // CRITICAL FIX: Use inPerTick (drive motors), NOT odoInPerTick (odometry wheels)
            double avgVelInchesPerSec = avgVel * AutoTankDrive.PARAMS.inPerTick;

            // Get battery voltage
            double voltage = voltageSensor.getVoltage();
            double voltageCommand = power * voltage;

            // Log to CSV as plain text
            try {
                if (csvWriter != null) {
                    csvWriter.write(String.format(Locale.US,
                            "%.4f,%.4f,%.4f,%.4f,%.1f,%.1f\n",
                            elapsedTime,
                            power,
                            voltageCommand,
                            avgVelInchesPerSec,
                            leftVel,
                            rightVel
                    ));
                    // Flush every 10 writes to ensure data is saved
                    if ((int)(elapsedTime * 100) % 10 == 0) {
                        csvWriter.flush();
                    }
                }
            } catch (IOException e) {
                telemetry.addLine("Warning: CSV write failed");
            }

            // Update telemetry
            telemetry.clearAll();
            telemetry.addLine("==================================");
            telemetry.addLine("RAMP TEST IN PROGRESS");
            telemetry.addLine("==================================");
            telemetry.addLine();
            telemetry.addData("Progress", "%.1f%%", (elapsedTime / RAMP_DURATION) * 100);
            telemetry.addData("Time Remaining", "%.1f sec", RAMP_DURATION - elapsedTime);
            telemetry.addLine();
            telemetry.addData("Current Power", "%.4f", power);
            telemetry.addData("Voltage Command", "%.3f V", voltageCommand);
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            telemetry.addLine();
            telemetry.addData("Avg Velocity", "%.1f ticks/sec", avgVel);
            telemetry.addData("Avg Velocity", "%.2f in/sec", avgVelInchesPerSec);
            telemetry.addLine();
            telemetry.addData("Left Velocity", "%.1f ticks/sec", leftVel);
            telemetry.addData("Right Velocity", "%.1f ticks/sec", rightVel);
            telemetry.update();

            sleep(10); // Small delay
        }

        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Close CSV file
        closeCSV();

        telemetry.clearAll();
        telemetry.addLine("==================================");
        telemetry.addLine("RAMP TEST COMPLETE");
        telemetry.addLine("==================================");
        telemetry.addLine();
        telemetry.addLine("✓ Data saved to: " + successPath);
        telemetry.addLine();
        telemetry.addLine("Next steps:");
        telemetry.addLine("1. Connect via USB or use adb:");
        telemetry.addLine("   adb pull " + successPath);
        telemetry.addLine("2. Or use FTC Dashboard file browser:");
        telemetry.addLine("   http://192.168.43.1:8080/dash");
        telemetry.addLine("   (Look in 'Files' tab)");
        telemetry.addLine("3. Or check Robot Controller app:");
        telemetry.addLine("   Menu > Manage > Download Logs");
        telemetry.addLine();
        telemetry.addLine("If file still not found, try:");
        telemetry.addLine("- Check /sdcard/FIRST/data/");
        telemetry.addLine("- Check /sdcard/Download/");
        telemetry.addLine("- Use file manager app on Control Hub");
        telemetry.addLine();
        telemetry.addLine("Analysis in Excel/Sheets:");
        telemetry.addLine("4. Create scatter plot:");
        telemetry.addLine("   X = velocityInchesPerSec");
        telemetry.addLine("   Y = voltageCommand");
        telemetry.addLine("5. Remove outliers near zero velocity");
        telemetry.addLine("6. Use formulas:");
        telemetry.addLine("   kV = SLOPE(voltageCommand, velocity)");
        telemetry.addLine("   kS = INTERCEPT(voltageCommand, velocity)");
        telemetry.addLine("7. Update AutoTankDrive.Params");
        telemetry.addLine();
        telemetry.addLine("Current values:");
        telemetry.addLine("kS: " + String.format("%.6f", AutoTankDrive.PARAMS.kS));
        telemetry.addLine("kV: " + String.format("%.6f", AutoTankDrive.PARAMS.kV));
        telemetry.update();

        sleep(30000); // Keep displaying for 30 seconds
    }


    private void closeCSV() {
        try {
            if (csvWriter != null) {
                csvWriter.flush();
                csvWriter.close();
                csvWriter = null;
            }
        } catch (IOException e) {
            telemetry.addLine("ERROR: Could not close CSV file");
            telemetry.addData("Exception", e.toString());
            telemetry.update();
        }
    }
}