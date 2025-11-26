package org.firstinspires.ftc.teamcode.champion.controller;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import java.util.List;

/**
 * Turret-based AprilTag alignment using zone-based PID control
 * Identifies AprilTag and rotates turret towards it
 */
@Config
public class TurretAlignmentController {

    private final LinearOpMode opMode;
    private final TurretController turretController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double ANGLE_TOLERANCE_DEGREES = 1.0;
        public static long ALIGNMENT_TIMEOUT_MS = 1000;
        public static int ALIGNED_FRAMES_REQUIRED = 5;
        public static double SETTLING_TIME_MS = 100;
    }

    @Config
    public static class PIDParams {
        // Zone 1: 0-5 degrees
        public static double ZONE1_KP = 8.0;
        public static double ZONE1_KD = 0.0;

        // Zone 2: 5-10 degrees
        public static double ZONE2_KP = 7.0;
        public static double ZONE2_KD = 0.0;

        // Zone 3: >10 degrees
        public static double ZONE3_KP = 6.0;
        public static double ZONE3_KD = 0.0;

        // Shared parameters
        public static double KI = 0.0;  // Not used

        // Output limits (power) - shared across all zones
        public static double MAX_POWER = 0.8;
        public static double MIN_POWER = 0.1;

        // Anti-windup
        public static double INTEGRAL_MAX = 30.0;

        // Deadband
        public static double DEADBAND_DEGREES = 0.5;
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_SPEED = 0.4;
        public static long MAX_SEARCH_TIME_MS = 5000;
    }

    // === ZONE DEFINITION ===

    public enum TXZone {
        ZONE1_CLOSE("0-5°"),      // 0-5 degrees
        ZONE2_MEDIUM("5-10°"),   // 5-10 degrees
        ZONE3_FAR(">10°");     // >10 degrees

        private final String description;

        TXZone(String description) {
            this.description = description;
        }

        public static TXZone getZone(double absTx) {
            if (absTx <= 5.0) return ZONE1_CLOSE;
            else if (absTx <= 10.0) return ZONE2_MEDIUM;
            else return ZONE3_FAR;
        }

        public String getDescription() {
            return description;
        }
    }

    // === PID CONTROLLER CLASS ===

    private class PIDController {
        private double setpoint = 0;
        private double lastError = 0;
        private double integral = 0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean firstUpdate = true;

        // Zone-specific PID gains (set once based on initial TX)
        private double kP = 6.0;
        private double kD = 0.0;

        public void reset() {
            lastError = 0;
            integral = 0;
            firstUpdate = true;
            timer.reset();
        }

        public void setTarget(double targetAngle) {
            this.setpoint = targetAngle;
            reset();
        }

        public void setZoneGains(TXZone zone) {
            switch (zone) {
                case ZONE1_CLOSE:
                    this.kP = PIDParams.ZONE1_KP;
                    this.kD = PIDParams.ZONE1_KD;
                    break;
                case ZONE2_MEDIUM:
                    this.kP = PIDParams.ZONE2_KP;
                    this.kD = PIDParams.ZONE2_KD;
                    break;
                case ZONE3_FAR:
                    this.kP = PIDParams.ZONE3_KP;
                    this.kD = PIDParams.ZONE3_KD;
                    break;
            }
        }

        public double calculate(double currentAngle) {
            // Calculate error (normalize to -180 to 180)
            double error = angleDifferenceDegrees(currentAngle, setpoint);

            // Apply deadband
            if (Math.abs(error) < PIDParams.DEADBAND_DEGREES) {
                return 0;
            }

            // Get time delta
            double dt = firstUpdate ? 0.02 : timer.seconds();
            timer.reset();

            if (firstUpdate) {
                firstUpdate = false;
                lastError = error;
            }

            // P term - using zone-specific kP
            double pTerm = kP * error;

            // I term (not used, KI = 0)
            integral += error * dt;
            integral = Math.max(-PIDParams.INTEGRAL_MAX,
                    Math.min(PIDParams.INTEGRAL_MAX, integral));
            double iTerm = PIDParams.KI * integral;

            // D term - using zone-specific kD
            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            double dTerm = kD * derivative;

            // Calculate total output
            double output = pTerm + iTerm + dTerm;

            // Apply output limits (shared across zones)
            output = Math.max(-PIDParams.MAX_POWER,
                    Math.min(PIDParams.MAX_POWER, output));

            // Apply minimum power if we're still moving
            if (Math.abs(output) > 0 && Math.abs(output) < PIDParams.MIN_POWER) {
                output = Math.copySign(PIDParams.MIN_POWER, output);
            }

            // Store for next iteration
            lastError = error;

            return output;
        }

        public double getError(double currentAngle) {
            return angleDifferenceDegrees(currentAngle, setpoint);
        }

        public boolean isAtSetpoint(double currentAngle) {
            return Math.abs(getError(currentAngle)) <= AlignmentParams.ANGLE_TOLERANCE_DEGREES;
        }

        public double getKP() {
            return kP;
        }

        public double getKD() {
            return kD;
        }
    }

    // === STATE VARIABLES ===

    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }

    private AlignmentState currentState = AlignmentState.STOPPED;
    private boolean isActive = false;

    // PID controller instance
    private final PIDController pidController = new PIDController();

    // Target tracking
    private int targetTagId = 20;
    private boolean hasTarget = false;
    private double initialTx = 0;
    private double initialTurretPosition = 0;
    private double targetTurretPosition = 0;
    private boolean targetTurretPositionCalculated = false;

    // Zone tracking
    private TXZone currentZone = TXZone.ZONE2_MEDIUM;

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();

    // === INITIALIZATION ===

    public TurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
        this.opMode = opMode;
        this.turretController = turretController;

        this.dashboard = FtcDashboard.getInstance();

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(1);
            this.limelight.start();
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }
    }

    // === PUBLIC API ===

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    public void startAlignment() {
        isActive = true;
        targetTurretPositionCalculated = false;
        alignmentTimer.reset();

        // Update turret position
        turretController.update();
        initialTurretPosition = turretController.getCurrentPosition();

        // Try to find target
        if (findTarget()) {
            // Determine zone based on absolute initial TX
            double absTx = Math.abs(initialTx);
            currentZone = TXZone.getZone(absTx);

            // Set PID gains based on zone
            pidController.setZoneGains(currentZone);

            // Calculate target turret position from initial tx
            targetTurretPosition = normalizeTurretAngle(initialTurretPosition + initialTx);
            targetTurretPositionCalculated = true;

            // Initialize PID controller with target
            pidController.setTarget(targetTurretPosition);

            opMode.telemetry.addLine("=== TURRET ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial TX", "%.2f°", initialTx);
            opMode.telemetry.addData("Abs TX", "%.2f°", absTx);
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addData("Initial Turret", "%.2f°", initialTurretPosition);
            opMode.telemetry.addData("Target Turret", "%.2f°", targetTurretPosition);
            opMode.telemetry.update();

            // Execute PID-controlled turn
            try {
                turnToAnglePID(targetTurretPosition);
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();

            opMode.telemetry.addLine("Target not found - starting search");
            opMode.telemetry.update();
        }

        consecutiveAlignedFrames = 0;
    }

    /**
     * Turn turret to target angle using PID control with zone-based gains
     * This is a BLOCKING method that returns when aligned or timeout
     */
    private void turnToAnglePID(double targetAngleDegrees) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        int stableFrames = 0;

        // PID gains are already set based on zone - they won't change during this turn
        pidController.setTarget(targetAngleDegrees);

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
                opMode.telemetry.addLine("⚠️ Alignment timeout");
                opMode.telemetry.update();
                break;
            }

            // Update turret position
            turretController.update();
            double currentAngle = turretController.getCurrentPosition();

            // Calculate PID output (power)
            double power = pidController.calculate(currentAngle);

            // Get current error for display
            double angleError = pidController.getError(currentAngle);

            // Display status
            opMode.telemetry.addLine("=== TURRET PID ALIGNMENT ===");
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Current", "%.2f°", currentAngle);
            opMode.telemetry.addData("Target", "%.2f°", targetAngleDegrees);
            opMode.telemetry.addData("Error", "%.2f°", angleError);
            opMode.telemetry.addData("Power", "%.3f", power);
            opMode.telemetry.addData("Stable", "%d/%d", stableFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);

            // Check if aligned
            if (pidController.isAtSetpoint(currentAngle)) {
                stableFrames++;
                opMode.telemetry.addLine("✓ Within tolerance");

                if (stableFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    turretController.setPower(0);
                    sleep((long)AlignmentParams.SETTLING_TIME_MS);
                    break;
                }

                // Stop while counting stable frames
                turretController.setPower(0);
            } else {
                // Reset stable counter if we drift
                stableFrames = 0;

                // Apply the PID-calculated power
                turretController.setPower(power);
            }

            opMode.telemetry.update();

            // Loop timing (50Hz)
            sleep(20);
        }

        // Ensure stopped
        turretController.setPower(0);
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        pidController.reset();

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        turretController.setPower(0);
    }

    public void maintainAlignment() {
        if (!targetTurretPositionCalculated) return;

        // Continuously apply PID to maintain turret position (uses same zone gains)
        turretController.update();
        double currentAngle = turretController.getCurrentPosition();

        // Use PID to calculate correction
        double power = pidController.calculate(currentAngle);

        // Apply correction if needed
        if (Math.abs(power) > PIDParams.DEADBAND_DEGREES) {
            turretController.setPower(power);
        } else {
            turretController.setPower(0);
        }
    }

    // === HELPER METHODS ===

    /**
     * Normalize angle to [-180, 180] range for turret
     * Considering turret limits from TurretController (0-144 degrees)
     */
    private double normalizeTurretAngle(double angle) {
        // First normalize to -180 to 180
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;

        // Then clamp to turret limits
        if (turretController.enableLimits) {
            angle = Math.max(turretController.minAngle,
                    Math.min(turretController.maxAngle, angle));
        }

        return angle;
    }

    /**
     * Calculate the smallest angle difference between two angles (in degrees)
     * Returns positive if target is CCW from current, negative if CW
     */
    private double angleDifferenceDegrees(double currentDegrees, double targetDegrees) {
        double diff = targetDegrees - currentDegrees;

        // Normalize to [-180, 180]
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;

        return diff;
    }

    /**
     * Find the target AprilTag and read its TX value
     */
    private boolean findTarget() {
        if (limelight == null) {
            return false;
        }

        try {
            // Take multiple readings for better accuracy
            double sumTx = 0;
            int validReadings = 0;

            for (int i = 0; i < 5; i++) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == targetTagId) {
                            sumTx += fiducial.getTargetXDegrees();
                            validReadings++;
                            hasTarget = true;
                            break;
                        }
                    }
                }

                try {
                    sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false;
                }
            }

            if (validReadings > 0) {
                initialTx = sumTx / validReadings;
                initialTx = -initialTx;  // Invert TX sign
                hasTarget = true;
                return true;
            }

        } catch (Exception ignored) {
        }

        hasTarget = false;
        return false;
    }

    private void executeSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);
            return;
        }

        // Try to find target during search
        if (findTarget()) {
            // Found target! Determine zone and set gains
            turretController.update();
            initialTurretPosition = turretController.getCurrentPosition();

            double absTx = Math.abs(initialTx);
            currentZone = TXZone.getZone(absTx);
            pidController.setZoneGains(currentZone);

            targetTurretPosition = normalizeTurretAngle(initialTurretPosition + initialTx);
            targetTurretPositionCalculated = true;
            hasTarget = true;

            try {
                turnToAnglePID(targetTurretPosition);
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
            return;
        }

        // Continue searching - slow rotation
        turretController.setClockwiseRotation();
    }

    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;

        switch (currentState) {
            case SEARCHING:
                executeSearch();
                break;
            case ALIGNED:
                maintainAlignment();
                break;
            case ALIGNING:
            case TARGET_LOST:
            case STOPPED:
                break;
        }

        sendDashboardData();
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Target_Calculated", targetTurretPositionCalculated);

        // Zone and PID parameters
        packet.put("Zone", currentZone.getDescription());
        packet.put("PID_KP", pidController.getKP());
        packet.put("PID_KD", pidController.getKD());

        // Angles
        turretController.update();
        double currentAngle = turretController.getCurrentPosition();
        packet.put("Initial_Turret", initialTurretPosition);
        packet.put("Current_Turret", currentAngle);
        packet.put("Target_Turret", targetTurretPosition);
        packet.put("Initial_TX", initialTx);
        packet.put("Abs_TX", Math.abs(initialTx));

        if (targetTurretPositionCalculated) {
            double error = angleDifferenceDegrees(currentAngle, targetTurretPosition);
            packet.put("Angle_Error", error);
        }

        // Power
        packet.put("Turret_Power", turretController.calculatePower(0)); // Assuming no manual input

        // Timing
        packet.put("Aligned_Frames", consecutiveAlignedFrames);
        packet.put("Alignment_Time", alignmentTimer.seconds());
        if (totalAlignmentTime > 0) {
            packet.put("Total_Time", totalAlignmentTime);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine("  TURRET ALIGNMENT");
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> INITIAL TX ANGLE <<<");
        opMode.telemetry.addData("TX", "%.2f°", initialTx);
        opMode.telemetry.addData("Abs TX", "%.2f°", Math.abs(initialTx));
        opMode.telemetry.addData("Zone", currentZone.getDescription());
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> PID GAINS (Active) <<<");
        opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
        opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);

        opMode.telemetry.update();
    }

    // === GETTERS ===

    public boolean isAligned() {
        return currentState == AlignmentState.ALIGNED;
    }

    public boolean isSearching() {
        return currentState == AlignmentState.SEARCHING;
    }

    public AlignmentState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTargetError() {
        if (!targetTurretPositionCalculated) return 999;
        turretController.update();
        return Math.abs(angleDifferenceDegrees(turretController.getCurrentPosition(), targetTurretPosition));
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public String getCurrentZone() {
        return currentZone.getDescription();
    }

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════╗");
        opMode.telemetry.addLine("║ TURRET ALIGNMENT      ║");
        opMode.telemetry.addLine("╚═══════════════════════╝");
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> INITIAL TX ANGLE <<<");
        opMode.telemetry.addData("TX", String.format(Locale.US, "%.2f°", initialTx));
        opMode.telemetry.addData("Abs TX", String.format(Locale.US, "%.2f°", Math.abs(initialTx)));
        opMode.telemetry.addData("Zone", currentZone.getDescription());
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> PID GAINS (Active) <<<");
        opMode.telemetry.addData("KP", String.format(Locale.US, "%.2f", pidController.getKP()));
        opMode.telemetry.addData("KD", String.format(Locale.US, "%.2f", pidController.getKD()));
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);

        opMode.telemetry.update();
    }
}