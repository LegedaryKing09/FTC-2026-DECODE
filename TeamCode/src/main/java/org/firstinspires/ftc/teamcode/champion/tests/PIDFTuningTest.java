package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "PIDF Tuning Test", group = "Testing")
public class PIDFTuningTest extends LinearOpMode {

    SixWheelDriveController driveController;

    @Config
    public static class TestParams {
        public static double TARGET_VELOCITY_PERCENT = 0.2; // 20% of max
        public static boolean RUN_STEP_TEST = true;
        public static boolean RUN_OSCILLATION_TEST = true;
        public static boolean RUN_BATTERY_TEST = true;
    }

    @Override
    public void runOpMode() {
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== PIDF TUNING TEST SUITE ===");
        telemetry.addLine();
        telemetry.addLine("This will run 3 tests:");
        telemetry.addLine("1. Step Response Test");
        telemetry.addLine("2. Oscillation Detection");
        telemetry.addLine("3. Battery Consistency");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // TEST 1: Step Response Test
        if (TestParams.RUN_STEP_TEST) {
            runStepResponseTest();
        }

        // TEST 2: Oscillation Detection
        if (TestParams.RUN_OSCILLATION_TEST) {
            runOscillationTest();
        }

        // TEST 3: Battery Voltage Consistency
        if (TestParams.RUN_BATTERY_TEST) {
            runBatteryConsistencyTest();
        }

        telemetry.clear();
        telemetry.addLine("=== ALL TESTS COMPLETE ===");
        telemetry.addLine("Review results above");
        telemetry.update();
    }

    /**
     * TEST 1: Step Response - Measures rise time, overshoot, settling time
     * IDEAL: Rise time < 0.3s, Overshoot < 10%, Settling time < 0.5s
     */
    private void runStepResponseTest() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  STEP RESPONSE TEST       ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("This test measures how quickly");
        telemetry.addLine("the motors reach target speed.");
        telemetry.addLine();
        telemetry.addLine("Press A to start");
        telemetry.update();

        while (!gamepad1.a && opModeIsActive()) {
            sleep(10);
        }

        double targetVel = TestParams.TARGET_VELOCITY_PERCENT *
                SixWheelDriveController.VelocityParams.MAX_TICKS_PER_SEC;

        ElapsedTime timer = new ElapsedTime();
        double riseTime = -1;
        double maxVelocity = 0;
        double settlingTime = -1;
        boolean hasReached90Percent = false;

        // Record data points
        List<Double> timestamps = new ArrayList<>();
        List<Double> velocities = new ArrayList<>();

        timer.reset();

        // Apply step input
        driveController.tankDriveVelocity(targetVel, targetVel);

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double currentVel = driveController.getLeftVelocity();
            double time = timer.seconds();

            timestamps.add(time);
            velocities.add(currentVel);

            // Measure rise time (time to reach 90% of target)
            if (!hasReached90Percent && currentVel >= targetVel * 0.9) {
                riseTime = time;
                hasReached90Percent = true;
            }

            // Track maximum velocity (for overshoot calculation)
            maxVelocity = Math.max(maxVelocity, currentVel);

            // Measure settling time (stays within 5% of target)
            if (hasReached90Percent && settlingTime < 0) {
                if (Math.abs(currentVel - targetVel) <= targetVel * 0.05) {
                    settlingTime = time;
                }
            }

            telemetry.addLine("=== RUNNING TEST ===");
            telemetry.addData("Time", String.format(Locale.US, "%.2fs", time));
            telemetry.addData("Target", String.format(Locale.US, "%.0f ticks/s", targetVel));
            telemetry.addData("Current", String.format(Locale.US, "%.0f ticks/s", currentVel));
            telemetry.addData("Error", String.format(Locale.US, "%.0f ticks/s", targetVel - currentVel));
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();

        // Calculate metrics
        double overshoot = ((maxVelocity - targetVel) / targetVel) * 100;

        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  STEP RESPONSE RESULTS    ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Rise Time (90%)", String.format(Locale.US, "%.3fs", riseTime));
        telemetry.addData("Overshoot", String.format(Locale.US, "%.1f%%", overshoot));
        telemetry.addData("Settling Time", String.format(Locale.US, "%.3fs", settlingTime));
        telemetry.addLine();
        telemetry.addLine("─── DIAGNOSIS ───");

        // Diagnosis
        if (riseTime < 0) {
            telemetry.addLine("❌ Never reached 90% - Check motors!");
        } else if (riseTime > 0.5) {
            telemetry.addLine("❌ Too slow - Increase P or F");
        } else if (riseTime < 0.1) {
            telemetry.addLine("⚠️ Very fast - Check for oscillation");
        } else {
            telemetry.addLine("✓ Rise time OK");
        }

        if (overshoot > 15) {
            telemetry.addLine("❌ Too much overshoot - Decrease P, increase D");
        } else if (overshoot < 0) {
            telemetry.addLine("⚠️ No overshoot - May be sluggish, increase P");
        } else {
            telemetry.addLine("✓ Overshoot OK");
        }

        if (settlingTime < 0 || settlingTime > 1.0) {
            telemetry.addLine("❌ Poor settling - Tune D term");
        } else {
            telemetry.addLine("✓ Settling time OK");
        }

        telemetry.addLine();
        telemetry.addLine("Press B to continue");
        telemetry.update();

        while (!gamepad1.b && opModeIsActive()) {
            sleep(10);
        }
    }

    /**
     * TEST 2: Oscillation Detection - Detects PID oscillation
     * IDEAL: No oscillation, steady velocity tracking
     */
    private void runOscillationTest() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  OSCILLATION TEST         ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("This test detects if the");
        telemetry.addLine("motors oscillate around target.");
        telemetry.addLine();
        telemetry.addLine("Press A to start");
        telemetry.update();

        while (!gamepad1.a && opModeIsActive()) {
            sleep(10);
        }

        double targetVel = TestParams.TARGET_VELOCITY_PERCENT *
                SixWheelDriveController.VelocityParams.MAX_TICKS_PER_SEC;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Wait for system to stabilize
        driveController.tankDriveVelocity(targetVel, targetVel);
        sleep(1000);

        // Measure velocity variance over 2 seconds
        List<Double> velocities = new ArrayList<>();
        int zeroCrossings = 0;
        double lastError = 0;

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double currentVel = driveController.getLeftVelocity();
            double error = targetVel - currentVel;

            velocities.add(currentVel);

            // Count zero crossings (sign changes in error)
            if (velocities.size() > 10) {
                if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
                    zeroCrossings++;
                }
            }
            lastError = error;

            telemetry.addLine("=== RUNNING TEST ===");
            telemetry.addData("Time", String.format(Locale.US, "%.2fs", timer.seconds()));
            telemetry.addData("Target", String.format(Locale.US, "%.0f", targetVel));
            telemetry.addData("Current", String.format(Locale.US, "%.0f", currentVel));
            telemetry.addData("Error", String.format(Locale.US, "%.0f", error));
            telemetry.addData("Zero Crossings", zeroCrossings);
            telemetry.update();

            sleep(50);
        }

        driveController.stopDrive();

        // Calculate standard deviation
        double mean = velocities.stream().mapToDouble(d -> d).average().orElse(0);
        double variance = velocities.stream()
                .mapToDouble(v -> Math.pow(v - mean, 2))
                .average().orElse(0);
        double stdDev = Math.sqrt(variance);
        double coefficientOfVariation = (stdDev / mean) * 100;

        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  OSCILLATION RESULTS      ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Mean Velocity", String.format(Locale.US, "%.0f ticks/s", mean));
        telemetry.addData("Std Deviation", String.format(Locale.US, "%.0f ticks/s", stdDev));
        telemetry.addData("Coeff. Variation", String.format(Locale.US, "%.1f%%", coefficientOfVariation));
        telemetry.addData("Zero Crossings", zeroCrossings);
        telemetry.addLine();
        telemetry.addLine("─── DIAGNOSIS ───");

        if (zeroCrossings > 20) {
            telemetry.addLine("❌ HIGH OSCILLATION - Reduce P, increase D");
        } else if (zeroCrossings > 10) {
            telemetry.addLine("⚠️ Moderate oscillation - Tune D term");
        } else {
            telemetry.addLine("✓ Stable control");
        }

        if (coefficientOfVariation > 10) {
            telemetry.addLine("❌ High variance - Increase D term");
        } else if (coefficientOfVariation > 5) {
            telemetry.addLine("⚠️ Some variance - Minor tuning needed");
        } else {
            telemetry.addLine("✓ Low variance");
        }

        telemetry.addLine();
        telemetry.addLine("Press B to continue");
        telemetry.update();

        while (!gamepad1.b && opModeIsActive()) {
            sleep(10);
        }
    }

    /**
     * TEST 3: Battery Consistency - Tests performance across voltage range
     * IDEAL: < 5% variation in velocity tracking across battery levels
     */
    private void runBatteryConsistencyTest() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  BATTERY CONSISTENCY TEST ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("This test runs at different");
        telemetry.addLine("speeds to verify consistency.");
        telemetry.addLine();
        telemetry.addLine("Press A to start");
        telemetry.update();

        while (!gamepad1.a && opModeIsActive()) {
            sleep(10);
        }

        double[] testVelocities = {0.25, 0.5, 0.75, 1.0}; // % of max
        List<String> results = new ArrayList<>();

        for (double percent : testVelocities) {
            double targetVel = percent * SixWheelDriveController.VelocityParams.MAX_TICKS_PER_SEC;

            driveController.tankDriveVelocity(targetVel, targetVel);
            sleep(1000); // Stabilization

            // Measure for 2 seconds
            List<Double> errors = new ArrayList<>();
            ElapsedTime timer = new ElapsedTime();

            while (timer.seconds() < 2.0 && opModeIsActive()) {
                double currentVel = driveController.getLeftVelocity();
                double error = Math.abs(targetVel - currentVel);
                errors.add(error);

                telemetry.addLine("=== RUNNING TEST ===");
                telemetry.addData("Test Speed", String.format(Locale.US, "%.0f%%", percent * 100));
                telemetry.addData("Target", String.format(Locale.US, "%.0f", targetVel));
                telemetry.addData("Current", String.format(Locale.US, "%.0f", currentVel));
                telemetry.addData("Error", String.format(Locale.US, "%.0f", error));
                telemetry.update();

                sleep(50);
            }

            double avgError = errors.stream().mapToDouble(d -> d).average().orElse(0);
            double percentError = (avgError / targetVel) * 100;

            results.add(String.format(Locale.US, "%.0f%% Speed: %.1f%% error",
                    percent * 100, percentError));

            driveController.stopDrive();
            sleep(500);
        }

        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║  BATTERY TEST RESULTS     ║");
        telemetry.addLine("╚═══════════════════════════╝");
        telemetry.addLine();

        for (String result : results) {
            telemetry.addLine(result);
        }

        telemetry.addLine();
        telemetry.addLine("─── DIAGNOSIS ───");
        telemetry.addLine("If errors increase at high speeds:");
        telemetry.addLine("  → Increase F term");
        telemetry.addLine("If errors vary widely:");
        telemetry.addLine("  → Tune P and I terms");
        telemetry.addLine();
        telemetry.addLine("Press B to finish");
        telemetry.update();

        while (!gamepad1.b && opModeIsActive()) {
            sleep(10);
        }
    }
}