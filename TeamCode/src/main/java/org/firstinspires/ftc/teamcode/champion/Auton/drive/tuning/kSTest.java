package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Static Friction Tuner (kS) - Autonomous Mode
 *
 * This OpMode helps you find the minimum voltage needed to overcome static friction.
 * Since your controller is broken, this runs automatically with adjustable parameters.
 *
 * Instructions:
 * 1. Open FTC Dashboard: http://192.168.43.1:8080/dash
 * 2. Adjust START_POWER on the dashboard (start low, like 0.05)
 * 3. Press INIT, then START
 * 4. Robot will apply that power and show if motors are moving
 * 5. Check telemetry - are wheels moving consistently?
 * 6. If NOT moving: increase START_POWER on dashboard, run again
 * 7. If MOVING: decrease START_POWER slightly, run again
 * 8. Repeat until you find the minimum power where wheels just start to move
 * 9. That power × battery voltage = kS
 *
 * Tips:
 * - Start very low (0.05) and increase gradually
 * - kS is typically between 0.01 and 0.15
 * - All wheels should start moving at roughly the same power
 * - The test runs for 10 seconds so you can observe the wheels
 */
@Config
@Autonomous(name = "Static Friction Tuner (kS) Auto", group = "Tuning")
public class kSTest extends LinearOpMode {

    // Adjustable on FTC Dashboard
    public static double START_POWER = 0.0978; // Adjust this value on the dashboard
    public static int TEST_DURATION_SECONDS = 10; // How long to run the test

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

        // Reset encoders
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

        telemetry.addLine("==================================");
        telemetry.addLine("STATIC FRICTION TUNER (kS)");
        telemetry.addLine("AUTONOMOUS MODE");
        telemetry.addLine("==================================");
        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Adjust START_POWER on Dashboard");
        telemetry.addLine("2. Press START");
        telemetry.addLine("3. Watch if wheels move");
        telemetry.addLine("4. Adjust and repeat until you find");
        telemetry.addLine("   minimum power for movement");
        telemetry.addLine();
        telemetry.addData("Current START_POWER", "%.4f", START_POWER);
        telemetry.addData("Test Duration", "%d seconds", TEST_DURATION_SECONDS);
        telemetry.addLine();
        telemetry.addLine("Dashboard:");
        telemetry.addLine("http://192.168.43.1:8080/dash");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Get initial battery voltage
        double batteryVoltage = voltageSensor.getVoltage();
        double calculatedKs = START_POWER * batteryVoltage;

        telemetry.clearAll();
        telemetry.addLine("==================================");
        telemetry.addLine("TEST STARTING");
        telemetry.addLine("==================================");
        telemetry.addData("Testing Power", "%.4f", START_POWER);
        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Calculated kS", "%.6f", calculatedKs);
        telemetry.addLine();
        telemetry.addLine("Watch the wheels...");
        telemetry.update();

        sleep(2000); // Give time to read

        long startTime = System.currentTimeMillis();
        long endTime = startTime + (TEST_DURATION_SECONDS * 1000);

        // Track if motors ever moved
        boolean everMoved = false;
        int movingCount = 0;
        int totalSamples = 0;

        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            // Apply power to motors
            leftFront.setPower(START_POWER);
            leftBack.setPower(START_POWER);
            rightFront.setPower(START_POWER);
            rightBack.setPower(START_POWER);

            // Get encoder velocities
            double lfVel = leftFront.getVelocity();
            double lbVel = leftBack.getVelocity();
            double rfVel = rightFront.getVelocity();
            double rbVel = rightBack.getVelocity();

            // Update battery voltage
            batteryVoltage = voltageSensor.getVoltage();
            calculatedKs = START_POWER * batteryVoltage;

            // Check if motors are moving (threshold: 10 ticks/sec)
            boolean lfMoving = Math.abs(lfVel) > 10;
            boolean lbMoving = Math.abs(lbVel) > 10;
            boolean rfMoving = Math.abs(rfVel) > 10;
            boolean rbMoving = Math.abs(rbVel) > 10;
            boolean allMoving = lfMoving && lbMoving && rfMoving && rbMoving;

            if (allMoving) {
                everMoved = true;
                movingCount++;
            }
            totalSamples++;

            // Calculate remaining time
            long remainingTime = (endTime - System.currentTimeMillis()) / 1000;

            // Display telemetry
            telemetry.clearAll();
            telemetry.addLine("==================================");
            telemetry.addLine("TESTING IN PROGRESS");
            telemetry.addLine("==================================");
            telemetry.addData("Time Remaining", "%d seconds", remainingTime);
            telemetry.addLine();

            telemetry.addLine("----------------------------------");
            telemetry.addLine("TEST PARAMETERS");
            telemetry.addLine("----------------------------------");
            telemetry.addData("Power Applied", "%.4f", START_POWER);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addData("Calculated kS", "%.6f", calculatedKs);
            telemetry.addLine();

            telemetry.addLine("----------------------------------");
            telemetry.addLine("MOTOR STATUS");
            telemetry.addLine("----------------------------------");
            telemetry.addData("Overall Status", allMoving ? "✓ ALL MOVING" : "✗ NOT ALL MOVING");
            telemetry.addLine();
            telemetry.addData("Left Front", lfMoving ? "✓ MOVING" : "✗ STOPPED", "%.0f t/s", lfVel);
            telemetry.addData("Left Back", lbMoving ? "✓ MOVING" : "✗ STOPPED", "%.0f t/s", lbVel);
            telemetry.addData("Right Front", rfMoving ? "✓ MOVING" : "✗ STOPPED", "%.0f t/s", rfVel);
            telemetry.addData("Right Back", rbMoving ? "✓ MOVING" : "✗ STOPPED", "%.0f t/s", rbVel);
            telemetry.addLine();

            telemetry.addLine("----------------------------------");
            telemetry.addLine("STATISTICS");
            telemetry.addLine("----------------------------------");
            double percentMoving = totalSamples > 0 ? (movingCount * 100.0 / totalSamples) : 0;
            telemetry.addData("Movement Consistency", "%.1f%%", percentMoving);
            telemetry.update();

            sleep(50);
        }

        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Final results
        telemetry.clearAll();
        telemetry.addLine("==================================");
        telemetry.addLine("TEST COMPLETE");
        telemetry.addLine("==================================");
        telemetry.addLine();

        telemetry.addLine("----------------------------------");
        telemetry.addLine("TEST RESULTS");
        telemetry.addLine("----------------------------------");
        telemetry.addData("Power Tested", "%.4f", START_POWER);
        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Calculated kS", "%.6f", calculatedKs);
        telemetry.addLine();

        double percentMoving = totalSamples > 0 ? (movingCount * 100.0 / totalSamples) : 0;
        telemetry.addData("Movement Consistency", "%.1f%%", percentMoving);
        telemetry.addLine();

        telemetry.addLine("----------------------------------");
        telemetry.addLine("INTERPRETATION");
        telemetry.addLine("----------------------------------");

        if (percentMoving > 90) {
            telemetry.addLine("✓ Wheels moved consistently!");
            telemetry.addLine();
            telemetry.addLine("Next step:");
            telemetry.addLine("→ DECREASE START_POWER slightly");
            telemetry.addLine("→ Run again to find minimum");
            telemetry.addLine();
            telemetry.addLine("Current kS candidate:");
            telemetry.addData("  kS", "%.6f", calculatedKs);
        } else if (percentMoving > 50) {
            telemetry.addLine("⚠ Wheels moved inconsistently");
            telemetry.addLine();
            telemetry.addLine("This might be close to kS!");
            telemetry.addLine("→ Try slightly HIGHER power");
            telemetry.addLine("   for consistent movement");
        } else if (percentMoving > 10) {
            telemetry.addLine("⚠ Wheels barely moved");
            telemetry.addLine();
            telemetry.addLine("→ INCREASE START_POWER");
            telemetry.addLine("→ Run again");
        } else {
            telemetry.addLine("✗ Wheels did not move");
            telemetry.addLine();
            telemetry.addLine("→ INCREASE START_POWER significantly");
            telemetry.addLine("→ Run again");
            telemetry.addLine();
        }

        telemetry.addLine();
        telemetry.addLine("----------------------------------");
        telemetry.addLine("NEXT STEPS");
        telemetry.addLine("----------------------------------");
        telemetry.addLine("1. Go to Dashboard");
        telemetry.addLine("2. Adjust START_POWER");
        telemetry.addLine("3. Run this OpMode again");
        telemetry.addLine("4. Goal: Find minimum power where");
        telemetry.addLine("   wheels move consistently (>90%)");
        telemetry.addLine();
        telemetry.addLine("When found, add to AutoTankDrive.Params:");
        telemetry.addLine("public double kS = " + String.format("%.6f", calculatedKs) + ";");

        telemetry.update();

        sleep(30000); // Display results for 30 seconds
    }
}