package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;

/**
 * Shooter PIDF Tuning Test with Intake/Transfer/Uptake
 * Uses NewShooterController (dual-motor with built-in PID)
 *
 * CONTROLS:
 * - A: Start/Stop shooter
 * - X: Switch to LOW preset (3800 RPM)
 * - Y: Switch to HIGH preset (4600 RPM)
 * - DPAD UP/DOWN: Fine adjust target RPM (+/- 50)
 * - B: Reset timing stats
 *
 * - RIGHT BUMPER: Toggle intake + transfer + uptake (all three)
 * - LEFT BUMPER: HOLD to VOMIT (reverse all)
 *
 * DASHBOARD:
 * - Graphs RPM, Target, Error, Power over time
 * - Tune kP, kI, kD, kF in NewShooterController via Dashboard
 *
 * TUNING PROCESS:
 * 1. Set kI=0, kD=0, kF=0 in NewShooterController config
 * 2. Find kF: Set target RPM, manually find power that maintains it
 *    kF ≈ power / targetRPM (e.g., 0.8 / 4000 = 0.0002)
 * 3. Tune kP: Increase until oscillates, back off 30%
 * 4. Tune kD: Add to stop oscillation (usually kD = kP * 0.01 to 0.1)
 * 5. Tune kI: Only if steady-state error persists (usually 0)
 */
@Config
@TeleOp(name = "Shooter PIDF Tuning", group = "Test")
public class ShooterPIDF extends OpMode {

    // ========== TUNABLE PARAMETERS (Dashboard) ==========
    public static double LOW_RPM = 3800.0;
    public static double HIGH_RPM = 4600.0;
    public static double RPM_INCREMENT = 50.0;

    // Tolerance for "at target" timing
    public static double RPM_TOLERANCE = 100.0;

    // ========== HARDWARE NAMES ==========
    public static String SHOOTER1_NAME = "shooter1";
    public static String SHOOTER2_NAME = "shooter2";
    public static String INTAKE_NAME = "intake";
    public static String TRANSFER_NAME = "transfer";
    public static String UPTAKE1_NAME = "servo1";
    public static String UPTAKE2_NAME = "servo2";

    // ========== INTAKE/TRANSFER/UPTAKE POWER ==========
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double UPTAKE_POWER = 1.0;

    // ========== SHOOTER (uses NewShooterController) ==========
    private NewShooterController shooter;

    // ========== INTAKE/TRANSFER/UPTAKE HARDWARE ==========
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private CRServo uptakeServo1;
    private CRServo uptakeServo2;

    // ========== STATE ==========
    private double targetRPM = HIGH_RPM;

    // ========== INTAKE MODE STATE ==========
    private boolean intakeModeActive = false;
    private boolean isVomiting = false;

    // ========== TIMING STATS ==========
    private ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean timing = false;
    private double lastSpinUpTime = 0;
    private double bestSpinUpTime = 999;
    private double worstSpinUpTime = 0;
    private int spinUpCount = 0;
    private double totalSpinUpTime = 0;

    // ========== RPM DROP TRACKING ==========
    private double minRPMDuringShot = 9999;
    private double rpmBeforeShot = 0;
    private boolean trackingDrop = false;

    // ========== DASHBOARD ==========
    private FtcDashboard dashboard;

    // ========== BUTTON DEBOUNCE ==========
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRB = false;

    @Override
    public void init() {
        // Initialize shooter via NewShooterController (dual motor)
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, SHOOTER1_NAME);
            shooterMotor2 = hardwareMap.get(DcMotor.class, SHOOTER2_NAME);
            telemetry.addData("Shooter", "OK (dual motor)");
        } catch (Exception e) {
            telemetry.addData("Shooter", "FAILED: " + e.getMessage());
        }
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);

        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("Intake", "FAILED");
            intakeMotor = null;
        }

        // Initialize transfer motor
        try {
            transferMotor = hardwareMap.get(DcMotor.class, TRANSFER_NAME);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("Transfer", "FAILED");
            transferMotor = null;
        }

        // Initialize uptake servos
        try {
            uptakeServo1 = hardwareMap.get(CRServo.class, UPTAKE1_NAME);
            uptakeServo2 = hardwareMap.get(CRServo.class, UPTAKE2_NAME);
            telemetry.addData("Uptake", "OK");
        } catch (Exception e) {
            telemetry.addData("Uptake", "FAILED");
            uptakeServo1 = null;
            uptakeServo2 = null;
        }

        // Initialize dashboard with merged telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== SHOOTER PIDF TUNING ===");
        telemetry.addLine("A: Start/Stop");
        telemetry.addLine("X: LOW preset  Y: HIGH preset");
        telemetry.addLine("DPAD: Fine adjust  B: Reset stats");
        telemetry.addLine("RB: Toggle feed  LB: Hold to VOMIT");
        telemetry.addLine("");
        telemetry.addLine("Tune PID gains in NewShooterController config");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== READ CONTROLS ==========
        handleControls();
        handleIntakeControls();

        // ========== UPDATE SHOOTER (runs PID internally) ==========
        shooter.update();

        // ========== UPDATE TIMING ==========
        updateTiming();

        // ========== TRACK RPM DROP ==========
        trackRPMDrop();

        // ========== SEND TO DASHBOARD ==========
        sendDashboardData();

        // ========== TELEMETRY ==========
        updateTelemetry();
    }

    private void handleControls() {
        // A - Start/Stop
        if (gamepad1.a && !lastA) {
            if (!shooter.isShootMode()) {
                shooter.setTargetRPM(targetRPM);
                shooter.startShooting();
                // Begin timing
                spinUpTimer.reset();
                timing = true;
            } else {
                shooter.stopShooting();
                timing = false;
            }
        }
        lastA = gamepad1.a;

        // X - LOW preset
        if (gamepad1.x && !lastX) {
            double oldTarget = targetRPM;
            targetRPM = LOW_RPM;
            shooter.setTargetRPM(targetRPM);
            if (shooter.isShootMode() && Math.abs(oldTarget - targetRPM) > 100) {
                // Switching presets - time the transition
                spinUpTimer.reset();
                timing = true;
            }
        }
        lastX = gamepad1.x;

        // Y - HIGH preset
        if (gamepad1.y && !lastY) {
            double oldTarget = targetRPM;
            targetRPM = HIGH_RPM;
            shooter.setTargetRPM(targetRPM);
            if (shooter.isShootMode() && Math.abs(oldTarget - targetRPM) > 100) {
                spinUpTimer.reset();
                timing = true;
            }
        }
        lastY = gamepad1.y;

        // DPAD UP - Increment
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetRPM = Math.min(targetRPM + RPM_INCREMENT, 6000);
            shooter.setTargetRPM(targetRPM);
        }
        lastDpadUp = gamepad1.dpad_up;

        // DPAD DOWN - Decrement
        if (gamepad1.dpad_down && !lastDpadDown) {
            targetRPM = Math.max(targetRPM - RPM_INCREMENT, 0);
            shooter.setTargetRPM(targetRPM);
        }
        lastDpadDown = gamepad1.dpad_down;

        // B - Reset stats
        if (gamepad1.b && !lastB) {
            resetStats();
        }
        lastB = gamepad1.b;
    }

    private void handleIntakeControls() {
        // Right bumper - toggle intake + transfer + uptake
        boolean currentRB = gamepad1.right_bumper;
        if (currentRB && !lastRB) {
            if (!intakeModeActive) {
                startIntakeMode();
            } else {
                stopIntakeMode();
            }
        }
        lastRB = currentRB;

        // Left bumper - VOMIT (hold to reverse)
        if (gamepad1.left_bumper) {
            if (!isVomiting) {
                isVomiting = true;
                setIntakePower(-INTAKE_POWER);
                setTransferPower(-TRANSFER_POWER);
                setUptakePower(-UPTAKE_POWER);
            }
        } else {
            if (isVomiting) {
                isVomiting = false;
                if (intakeModeActive) {
                    setIntakePower(INTAKE_POWER);
                    setTransferPower(TRANSFER_POWER);
                    setUptakePower(UPTAKE_POWER);
                } else {
                    setIntakePower(0);
                    setTransferPower(0);
                    setUptakePower(0);
                }
            }
        }
    }

    private void startIntakeMode() {
        intakeModeActive = true;

        // Record RPM before feeding balls
        rpmBeforeShot = shooter.getRPM();
        minRPMDuringShot = shooter.getRPM();
        trackingDrop = true;

        setIntakePower(INTAKE_POWER);
        setTransferPower(TRANSFER_POWER);
        setUptakePower(UPTAKE_POWER);
    }

    private void stopIntakeMode() {
        intakeModeActive = false;
        trackingDrop = false;

        setIntakePower(0);
        setTransferPower(0);
        setUptakePower(0);
    }

    private void setIntakePower(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }

    private void setTransferPower(double power) {
        if (transferMotor != null) transferMotor.setPower(power);
    }

    private void setUptakePower(double power) {
        if (uptakeServo1 != null) uptakeServo1.setPower(power);
        if (uptakeServo2 != null) uptakeServo2.setPower(power);
    }

    private void trackRPMDrop() {
        if (trackingDrop && shooter.isShootMode()) {
            double rpm = shooter.getRPM();
            if (rpm < minRPMDuringShot) {
                minRPMDuringShot = rpm;
            }
        }
    }

    private void updateTiming() {
        if (!timing) return;

        double error = Math.abs(targetRPM - shooter.getRPM());

        if (error <= RPM_TOLERANCE) {
            double timeToReach = spinUpTimer.seconds();
            timing = false;

            lastSpinUpTime = timeToReach;
            spinUpCount++;
            totalSpinUpTime += timeToReach;

            if (timeToReach < bestSpinUpTime) bestSpinUpTime = timeToReach;
            if (timeToReach > worstSpinUpTime) worstSpinUpTime = timeToReach;
        }
    }

    private void resetStats() {
        bestSpinUpTime = 999;
        worstSpinUpTime = 0;
        spinUpCount = 0;
        totalSpinUpTime = 0;
        lastSpinUpTime = 0;
        minRPMDuringShot = 9999;
        rpmBeforeShot = 0;
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        double currentRPM = shooter.getRPM();
        double error = targetRPM - currentRPM;

        // Main graph values — these plot as time-series in FTC Dashboard
        packet.put("RPM", currentRPM);
        packet.put("Target", targetRPM);
        packet.put("Error", error);
        packet.put("Power", shooter.getCurrentPower());

        // PID terms (read from controller gains for visibility)
        packet.put("P_term", NewShooterController.kP * error);
        packet.put("I_term", NewShooterController.kI * shooter.getIntegralSum());
        packet.put("FF_term", NewShooterController.kF * targetRPM);

        // Timing & state
        packet.put("SpinUpTime", lastSpinUpTime);
        packet.put("Running", shooter.isShootMode() ? 1 : 0);

        // Intake/feeding state
        packet.put("Feeding", intakeModeActive ? 1 : 0);
        packet.put("MinRPM", minRPMDuringShot < 9999 ? minRPMDuringShot : 0);
        packet.put("RPMDrop", rpmBeforeShot - (minRPMDuringShot < 9999 ? minRPMDuringShot : rpmBeforeShot));

        dashboard.sendTelemetryPacket(packet);
    }

    private void updateTelemetry() {
        double currentRPM = shooter.getRPM();

        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Running", shooter.isShootMode() ? "YES" : "NO");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Error", "%.0f", targetRPM - currentRPM);
        telemetry.addData("Power", "%.3f", shooter.getCurrentPower());

        telemetry.addLine("");
        telemetry.addLine("=== INTAKE/FEEDING ===");
        telemetry.addData("Intake Mode", intakeModeActive ? "ACTIVE" : "OFF");
        telemetry.addData("Vomiting", isVomiting ? "YES" : "NO");
        if (minRPMDuringShot < 9999) {
            telemetry.addData("RPM Before Shot", "%.0f", rpmBeforeShot);
            telemetry.addData("Min RPM During Shot", "%.0f", minRPMDuringShot);
            telemetry.addData("RPM Drop", "%.0f", rpmBeforeShot - minRPMDuringShot);
        }

        telemetry.addLine("");
        telemetry.addLine("=== PIDF GAINS (edit in NewShooterController) ===");
        telemetry.addData("kF", "%.6f", NewShooterController.kF);
        telemetry.addData("kP", "%.6f", NewShooterController.kP);
        telemetry.addData("kI", "%.6f", NewShooterController.kI);
        telemetry.addData("kD", "%.6f", NewShooterController.kD);

        telemetry.addLine("");
        telemetry.addLine("=== TIMING STATS ===");
        telemetry.addData("Last Time", "%.3f sec", lastSpinUpTime);
        telemetry.addData("Best Time", "%.3f sec", bestSpinUpTime < 999 ? bestSpinUpTime : 0);
        telemetry.addData("Worst Time", "%.3f sec", worstSpinUpTime);
        telemetry.addData("Avg Time", "%.3f sec", spinUpCount > 0 ? totalSpinUpTime / spinUpCount : 0);
        telemetry.addData("Count", spinUpCount);

        if (timing) {
            telemetry.addLine("");
            telemetry.addData("TIMING", "%.3f sec...", spinUpTimer.seconds());
        }

        telemetry.addLine("");
        telemetry.addLine("A=Start/Stop  X=LOW  Y=HIGH");
        telemetry.addLine("RB=Feed  LB=Vomit  B=Reset");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stopShooting();
        setIntakePower(0);
        setTransferPower(0);
        setUptakePower(0);
    }
}