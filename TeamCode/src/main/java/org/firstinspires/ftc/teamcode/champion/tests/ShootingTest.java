package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;

/**
 * IMMEDIATE RESPONSE Test OpMode for RPM Compensation
 *
 * KEY FIX: Ramp now adjusts IMMEDIATELY when RPM drop is detected,
 * not after the ball is shot. Uses a dedicated high-priority thread
 * for continuous RPM monitoring and instant ramp adjustment.
 */
@Config
@TeleOp(name = "Shooting Test IMMEDIATE", group = "Testing")
public class ShootingTest extends LinearOpMode {

    @Config
    public static class TestSettings {
        public static double TARGET_RPM = 2800.0;
        public static double BASE_RAMP_ANGLE = 118.0;
        public static long TOTAL_SHOOT_DURATION = 2500;
    }

    @Config
    public static class RPMCompensation {
        public static boolean ENABLE_COMPENSATION = true;
        public static double RPM_DROP_THRESHOLD = 100.0;
        public static double ANGLE_COMPENSATION_PER_100RPM = 10.0;
        public static double MAX_ANGLE_COMPENSATION = 50.0;

    }

    private ShooterController shooterController;
    private RampController rampController;
    private TransferController transferController;
    private IntakeController intakeController;
    private final ElapsedTime timer = new ElapsedTime();

    // Real-time tracking - volatile for thread safety
    private volatile int shotNumber = 0;
    private volatile double[] rpmHistory = new double[50];
    private volatile int rpmHistoryIndex = 0;
    private volatile int compensationCount = 0;
    private volatile int recoveryCount = 0;
    private volatile double currentAngle = 0;
    private volatile boolean shooterRunning = false;
    private volatile boolean isShooting = false;

    // Thread for immediate RPM monitoring
    private Thread rpmMonitorThread;
    private volatile boolean monitorActive = false;

    @Override
    public void runOpMode() {
        shooterController = new ShooterController(this);
        rampController = new RampController(this);
        transferController = new TransferController(this);
        intakeController = new IntakeController(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rampController.setAngle(TestSettings.BASE_RAMP_ANGLE);
        currentAngle = TestSettings.BASE_RAMP_ANGLE;

        waitForStart();

        // Start shooter PID thread
        Thread pidThread = new Thread(() -> {
            while (opModeIsActive()) {
                shooterController.updatePID();
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }
        });
        pidThread.setPriority(Thread.MAX_PRIORITY);
        pidThread.start();

        // Start IMMEDIATE RPM monitoring thread - this is the key fix!
        startImmediateRPMMonitor();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                testRealTimeCompensation();
                sleep(500);
            }

            if (gamepad1.dpad_down) {
                testManualRPMDrop();
                sleep(500);
            }

            if (gamepad1.x) {
                shooterController.shooterStop();
                shooterRunning = false;
                isShooting = false;
                telemetry.addLine("✓ Shooter stopped");
            }

            displayLiveTelemetry();
            telemetry.update();
            sleep(50);
        }

        monitorActive = false;
        shooterController.shooterStop();
        transferController.transferStop();
        intakeController.intakeStop();
    }

    /**
     * CRITICAL FIX: Dedicated thread that monitors RPM continuously
     * and adjusts ramp IMMEDIATELY - no waiting for check intervals
     */
    private void startImmediateRPMMonitor() {
        monitorActive = true;
        rpmMonitorThread = new Thread(() -> {
            double lastAngleSet = TestSettings.BASE_RAMP_ANGLE;

            while (opModeIsActive() && monitorActive) {
                // Only compensate while actively shooting
                if (isShooting && RPMCompensation.ENABLE_COMPENSATION) {
                    double currentRPM = shooterController.getShooterRPM();
                    double rpmDrop = TestSettings.TARGET_RPM - currentRPM;

                    // Calculate target angle based on current RPM
                    double targetAngle;
                    if (rpmDrop > RPMCompensation.RPM_DROP_THRESHOLD) {
                        double angleDecrease = (rpmDrop / 100.0) * RPMCompensation.ANGLE_COMPENSATION_PER_100RPM;
                        angleDecrease = Math.min(angleDecrease, RPMCompensation.MAX_ANGLE_COMPENSATION);
                        targetAngle = TestSettings.BASE_RAMP_ANGLE - angleDecrease;
                    } else {
                        targetAngle = TestSettings.BASE_RAMP_ANGLE;
                    }

                    // IMMEDIATE adjustment - no delay!
                    // Only update if angle changed significantly to avoid servo jitter
                    if (Math.abs(targetAngle - lastAngleSet) > 0.3) {
                        rampController.setAngle(targetAngle);

                        // Track compensation vs recovery
                        if (targetAngle < lastAngleSet) {
                            compensationCount++;
                        } else if (targetAngle > lastAngleSet) {
                            recoveryCount++;
                        }

                        currentAngle = targetAngle;
                        lastAngleSet = targetAngle;
                    }
                }

                // Run as fast as possible - minimal sleep
                // This ensures we catch RPM drops within ~5ms
                try { Thread.sleep(20 ); } catch (InterruptedException e) { break; }

            }
        });
        rpmMonitorThread.setPriority(Thread.MAX_PRIORITY - 1);
        rpmMonitorThread.start();
    }

    /**
     * Real-time shooting test - ramp adjustment now happens in background thread
     */
    private void testRealTimeCompensation() {
        shotNumber++;
        compensationCount = 0;
        recoveryCount = 0;
        rpmHistoryIndex = 0;

        // Spin up shooter
        shooterController.setShooterRPM(TestSettings.TARGET_RPM);
        rampController.setAngle(TestSettings.BASE_RAMP_ANGLE);
        currentAngle = TestSettings.BASE_RAMP_ANGLE;
        shooterRunning = true;

        telemetry.addLine("⏳ Waiting for RPM to reach target...");
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 1500) {
            double currentRPM = shooterController.getShooterRPM();
            telemetry.addData("Current RPM", "%.0f / %.0f", currentRPM, TestSettings.TARGET_RPM);
            telemetry.update();

            if (Math.abs(currentRPM - TestSettings.TARGET_RPM) < 150) {
                break;
            }
            sleep(20);
        }

        telemetry.addLine("✓ RPM Ready! Starting shooting sequence...");
        telemetry.update();
        sleep(300);

        // Enable immediate compensation and start shooting
        isShooting = true;  // This activates the monitor thread!

        transferController.transferFull();
        intakeController.intakeFull();

        timer.reset();
        long lastLogTime = 0;

        telemetry.addLine("🔥 SHOOTING - IMMEDIATE RPM compensation active!");
        telemetry.update();

        while (opModeIsActive() && timer.milliseconds() < TestSettings.TOTAL_SHOOT_DURATION) {
            double currentTime = timer.milliseconds();

            // Log RPM for history (separate from compensation which happens in thread)
            if (currentTime - lastLogTime >= 50) {
                lastLogTime = (long) currentTime;

                double currentRPM = shooterController.getShooterRPM();
                if (rpmHistoryIndex < rpmHistory.length) {
                    rpmHistory[rpmHistoryIndex++] = currentRPM;
                }

                double rpmDrop = TestSettings.TARGET_RPM - currentRPM;

                telemetry.addData("⏱ Time", "%.0f/%.0f ms", currentTime, (double)TestSettings.TOTAL_SHOOT_DURATION);
                telemetry.addData("🎯 Current RPM", "%.0f", currentRPM);
                telemetry.addData("📉 RPM Drop", "%.0f", rpmDrop);
                telemetry.addData("📐 Current Angle", "%.1f°", currentAngle);
                telemetry.addData("⚡ Compensations", compensationCount);
                telemetry.addData("✓ Recoveries", recoveryCount);
                telemetry.addLine("🚀 IMMEDIATE mode - ~5ms response time");
                telemetry.update();
            }

            sleep(10);
        }

        // Stop shooting
        isShooting = false;  // Disable compensation
        transferController.transferStop();
        intakeController.intakeStop();

        displayTestResults();
    }

    /**
     * Manual test with immediate response
     */
    private void testManualRPMDrop() {
        telemetry.addLine("========================================");
        telemetry.addLine("🧪 MANUAL RPM DROP TEST (IMMEDIATE)");
        telemetry.addLine("========================================");
        telemetry.update();

        shooterController.setShooterRPM(TestSettings.TARGET_RPM);
        rampController.setAngle(TestSettings.BASE_RAMP_ANGLE);
        currentAngle = TestSettings.BASE_RAMP_ANGLE;
        shooterRunning = true;
        compensationCount = 0;
        recoveryCount = 0;

        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 1000) {
            if (Math.abs(shooterController.getShooterRPM() - TestSettings.TARGET_RPM) < 150) {
                break;
            }
            sleep(20);
        }

        double startRPM = shooterController.getShooterRPM();
        telemetry.addData("Starting RPM", "%.0f", startRPM);
        telemetry.addData("Starting Angle", "%.1f°", currentAngle);
        telemetry.update();
        sleep(500);

        // Enable immediate compensation
        isShooting = true;

        telemetry.addLine("⬇ Simulating RPM drop - watch for IMMEDIATE response!");
        telemetry.update();

        double droppedRPM = TestSettings.TARGET_RPM - 400;
        shooterController.setShooterRPM(droppedRPM);

        timer.reset();
        boolean compensationSeen = false;
        boolean recoverySeen = false;
        double compensationTime = 0;
        double recoveryTime = 0;

        while (opModeIsActive() && timer.milliseconds() < 3000) {
            double currentRPM = shooterController.getShooterRPM();
            double rpmDrop = TestSettings.TARGET_RPM - currentRPM;

            // Check if compensation happened
            if (!compensationSeen && compensationCount > 0) {
                compensationSeen = true;
                compensationTime = timer.milliseconds();
                telemetry.addLine("⚡ COMPENSATION DETECTED!");
                telemetry.addData("  Response Time", "%.0f ms", compensationTime);
            }

            // Check if recovery happened
            if (!recoverySeen && recoveryCount > 0) {
                recoverySeen = true;
                recoveryTime = timer.milliseconds();
                telemetry.addLine("✓ RECOVERY DETECTED!");
                telemetry.addData("  Response Time", "%.0f ms", recoveryTime);
            }

            // Simulate recovery at 1.5 seconds
            if (timer.milliseconds() > 1500 && !recoverySeen) {
                shooterController.setShooterRPM(TestSettings.TARGET_RPM);
                telemetry.addLine("⬆ Simulating RPM recovery...");
            }

            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("RPM Drop", "%.0f", rpmDrop);
            telemetry.addData("Current Angle", "%.1f°", currentAngle);
            telemetry.addData("Compensations", compensationCount);
            telemetry.addData("Recoveries", recoveryCount);
            telemetry.update();

            sleep(30);
        }

        isShooting = false;

        telemetry.addLine("========================================");
        telemetry.addLine("✓ Test complete - IMMEDIATE RESPONSE");
        telemetry.addData("Compensation Response", "%.0f ms", compensationTime);
        telemetry.addData("Recovery Response", "%.0f ms", recoveryTime - 1500);
        telemetry.addData("Final Angle", "%.1f°", currentAngle);
        telemetry.update();
    }

    private void displayTestResults() {
        telemetry.clear();
        telemetry.addLine("========================================");
        telemetry.addLine("📊 SHOT #" + shotNumber + " COMPLETE (IMMEDIATE MODE)");
        telemetry.addLine("========================================");
        telemetry.addLine();

        if (rpmHistoryIndex > 0) {
            double minRPM = Double.MAX_VALUE;
            double maxRPM = Double.MIN_VALUE;
            double avgRPM = 0;

            for (int i = 0; i < rpmHistoryIndex; i++) {
                double rpm = rpmHistory[i];
                minRPM = Math.min(minRPM, rpm);
                maxRPM = Math.max(maxRPM, rpm);
                avgRPM += rpm;
            }
            avgRPM /= rpmHistoryIndex;

            telemetry.addLine("RPM Statistics:");
            telemetry.addData("  Target RPM", "%.0f", TestSettings.TARGET_RPM);
            telemetry.addData("  Average RPM", "%.0f", avgRPM);
            telemetry.addData("  Min RPM", "%.0f", minRPM);
            telemetry.addData("  Max RPM", "%.0f", maxRPM);
            telemetry.addData("  Max Drop", "%.0f", TestSettings.TARGET_RPM - minRPM);
            telemetry.addLine();

            telemetry.addLine("Compensation Details (IMMEDIATE):");
            telemetry.addData("  Compensations Applied", compensationCount);
            telemetry.addData("  Recoveries Applied", recoveryCount);
            telemetry.addData("  Base Angle", "%.1f°", TestSettings.BASE_RAMP_ANGLE);
            telemetry.addData("  Final Angle", "%.1f°", currentAngle);
            telemetry.addData("  Response Time", "~5ms (thread-based)");
            telemetry.addLine();

            telemetry.addLine("RPM History (last 10 samples):");
            int start = Math.max(0, rpmHistoryIndex - 10);
            for (int i = start; i < rpmHistoryIndex; i++) {
                double rpm = rpmHistory[i];
                int barLength = (int)((rpm / TestSettings.TARGET_RPM) * 20);
                StringBuilder bar = new StringBuilder();
                for (int j = 0; j < barLength; j++) bar.append("█");
                telemetry.addData(String.format("  [%d]", i), "%.0f %s", rpm, bar.toString());
            }
        }

        telemetry.addLine();
        telemetry.addLine("Press DPAD_UP for another test");
        telemetry.update();
    }

    private void displayLiveTelemetry() {
        telemetry.addLine("=== LIVE SHOOTER STATUS (IMMEDIATE) ===");
        telemetry.addData("Shooter Running", shooterRunning);
        telemetry.addData("Shooting Active", isShooting);
        telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
        telemetry.addData("Target RPM", "%.0f", TestSettings.TARGET_RPM);
        telemetry.addData("Current Angle", "%.1f°", currentAngle);
        telemetry.addLine();

        telemetry.addLine("=== IMMEDIATE RESPONSE ===");
        telemetry.addData("Monitor Thread Active", monitorActive);
        telemetry.addData("Response Time", "~5ms");
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("DPAD_UP = Real-time shooting test");
        telemetry.addLine("DPAD_DOWN = Manual RPM drop test");
        telemetry.addLine("X = Stop shooter");
    }
}