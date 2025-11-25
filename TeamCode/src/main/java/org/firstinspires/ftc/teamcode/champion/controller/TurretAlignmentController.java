package org.firstinspires.ftc.teamcode.champion.controller;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Turret alignment controller using Limelight AprilTag detection
 * Aligns turret to center on target by compensating for TX error
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
        public static double TX_TOLERANCE_DEGREES = 2.0;      // How close is "aligned"
        public static long ALIGNMENT_TIMEOUT_MS = 3000;       // Max time for alignment
        public static int ALIGNED_FRAMES_REQUIRED = 5;        // Stable frames needed
        public static double SETTLING_TIME_MS = 100;          // Settling after aligned
        public static int LIMELIGHT_SAMPLES = 3;              // Samples to average
    }

    @Config
    public static class PIDParams {
        // Zone 1: 0-5 degrees TX error
        public static double ZONE1_KP = 1.2;
        public static double ZONE1_KD = 0.05;

        // Zone 2: 5-15 degrees TX error
        public static double ZONE2_KP = 1.0;
        public static double ZONE2_KD = 0.08;

        // Zone 3: >15 degrees TX error
        public static double ZONE3_KP = 0.8;
        public static double ZONE3_KD = 0.1;

        // Shared parameters
        public static double KI = 0.0;  // Integral (typically not needed)

        // Output limits (degrees to move turret)
        public static double MAX_POSITION_CHANGE = 30.0;      // Max degrees per update
        public static double MIN_POSITION_CHANGE = 1.0;       // Min movement to overcome friction

        // Anti-windup
        public static double INTEGRAL_MAX = 20.0;

        // Deadband
        public static double DEADBAND_DEGREES = 0.5;
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_INCREMENT_DEGREES = 10.0;  // How much to rotate each step
        public static long SEARCH_WAIT_MS = 300;               // Wait time between increments
        public static long MAX_SEARCH_TIME_MS = 10000;         // Max total search time
    }

    // === ZONE DEFINITION ===

    public enum TXZone {
        ZONE1_CLOSE("0-5°"),      // 0-5 degrees
        ZONE2_MEDIUM("5-15°"),    // 5-15 degrees
        ZONE3_FAR(">15°");        // >15 degrees

        private final String description;

        TXZone(String description) {
            this.description = description;
        }

        public static TXZone getZone(double absTx) {
            if (absTx <= 5.0) return ZONE1_CLOSE;
            else if (absTx <= 15.0) return ZONE2_MEDIUM;
            else return ZONE3_FAR;
        }

        public String getDescription() {
            return description;
        }
    }

    // === PID CONTROLLER CLASS ===

    private static class PIDController {
        private double lastError = 0;
        private double integral = 0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean firstUpdate = true;

        // Zone-specific PID gains
        private double kP = 1.0;
        private double kD = 0.05;

        public void reset() {
            lastError = 0;
            integral = 0;
            firstUpdate = true;
            timer.reset();
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

        /**
         * Calculate position change needed based on TX error
         * @param txError Current TX error from Limelight (negative TX = target is left)
         * @return Position change in degrees (positive = rotate right)
         */
        public double calculate(double txError) {
            // Apply deadband
            if (Math.abs(txError) < PIDParams.DEADBAND_DEGREES) {
                return 0;
            }

            // Get time delta
            double dt = firstUpdate ? 0.02 : timer.seconds();
            timer.reset();

            if (firstUpdate) {
                firstUpdate = false;
                lastError = txError;
            }

            // P term - proportional to error
            // If TX is negative (target left), we want positive position change (rotate right)
            // So we negate the TX error
            double pTerm = kP * (-txError);

            // I term (typically not used)
            integral += txError * dt;
            integral = Math.max(-PIDParams.INTEGRAL_MAX,
                    Math.min(PIDParams.INTEGRAL_MAX, integral));
            double iTerm = PIDParams.KI * (-integral);

            // D term - rate of change
            double derivative = dt > 0 ? (txError - lastError) / dt : 0;
            double dTerm = kD * (-derivative);

            // Calculate total output (position change in degrees)
            double output = pTerm + iTerm + dTerm;

            // Apply output limits
            output = Math.max(-PIDParams.MAX_POSITION_CHANGE,
                    Math.min(PIDParams.MAX_POSITION_CHANGE, output));

            // Apply minimum movement if we're correcting
            if (Math.abs(output) > 0 && Math.abs(output) < PIDParams.MIN_POSITION_CHANGE) {
                output = Math.copySign(PIDParams.MIN_POSITION_CHANGE, output);
            }

            // Store for next iteration
            lastError = txError;

            return output;
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
    private double currentTx = 0;

    // Zone tracking
    private TXZone currentZone = TXZone.ZONE2_MEDIUM;

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();
    private final ElapsedTime searchWaitTimer = new ElapsedTime();
    private int searchDirection = 1;  // 1 = right, -1 = left

    // === INITIALIZATION ===

    public TurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
        this.opMode = opMode;
        this.turretController = turretController;
        this.dashboard = FtcDashboard.getInstance();

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(1);  // AprilTag pipeline
            this.limelight.start();
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }
    }

    // === PUBLIC API ===

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    /**
     * Start alignment process
     */
    public void startAlignment() {
        isActive = true;
        alignmentTimer.reset();
        consecutiveAlignedFrames = 0;

        // Try to find target
        if (findTarget()) {
            currentState = AlignmentState.ALIGNING;

            // Determine zone based on absolute TX
            double absTx = Math.abs(currentTx);
            currentZone = TXZone.getZone(absTx);

            // Set PID gains based on zone
            pidController.setZoneGains(currentZone);

            opMode.telemetry.addLine("=== TURRET ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial TX", "%.2f°", currentTx);
            opMode.telemetry.addData("Abs TX", "%.2f°", absTx);
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addData("Turret Pos", "%.1f°", turretController.getCurrentPosition());
            opMode.telemetry.update();

            // Execute alignment
            try {
                executeAlignment();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
        } else {
            // No target found - start search
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            searchWaitTimer.reset();
            searchDirection = 1;

            opMode.telemetry.addLine("Target not found - starting search");
            opMode.telemetry.update();
        }
    }

    /**
     * Execute the alignment process (BLOCKING method)
     * @noinspection BusyWait
     */
    private void executeAlignment() throws InterruptedException {
        long startTime = System.currentTimeMillis();
        consecutiveAlignedFrames = 0;

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
                opMode.telemetry.addLine("⚠️ Alignment timeout");
                opMode.telemetry.update();
                currentState = AlignmentState.TARGET_LOST;
                break;
            }

            // Try to get fresh target data
            if (!findTarget()) {
                // Lost target
                opMode.telemetry.addLine("⚠️ Target lost");
                opMode.telemetry.update();
                currentState = AlignmentState.TARGET_LOST;
                break;
            }

            // Update zone if TX error changes significantly
            double absTx = Math.abs(currentTx);
            TXZone newZone = TXZone.getZone(absTx);
            if (newZone != currentZone) {
                currentZone = newZone;
                pidController.setZoneGains(currentZone);
            }

            // Calculate position change using PID
            double positionChange = pidController.calculate(currentTx);

            // Display status
            opMode.telemetry.addLine("=== TURRET PID ALIGNMENT ===");
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addLine();
            opMode.telemetry.addData("TX Error", "%.2f°", currentTx);
            opMode.telemetry.addData("Abs Error", "%.2f°", absTx);
            opMode.telemetry.addData("Position Change", "%.2f°", positionChange);
            opMode.telemetry.addData("Turret Pos", "%.1f°", turretController.getCurrentPosition());
            opMode.telemetry.addData("Stable", "%d/%d", consecutiveAlignedFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);

            // Check if aligned (within tolerance)
            if (absTx <= AlignmentParams.TX_TOLERANCE_DEGREES) {
                consecutiveAlignedFrames++;
                opMode.telemetry.addLine("✓ Within tolerance");

                if (consecutiveAlignedFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    currentState = AlignmentState.ALIGNED;
                    totalAlignmentTime = alignmentTimer.seconds();
                    sleep((long)AlignmentParams.SETTLING_TIME_MS);
                    break;
                }
            } else {
                // Reset stable counter if we drift
                consecutiveAlignedFrames = 0;

                // Move turret by calculated amount
                double newTarget = turretController.getCurrentPosition() + positionChange;
                turretController.setTargetPosition(newTarget);
            }

            opMode.telemetry.update();

            // Update turret controller
            turretController.update();

            // Loop timing (50Hz)
            sleep(20);
        }
    }

    /**
     * Stop alignment
     */
    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        pidController.reset();

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        turretController.stop();
    }

    /**
     * Maintain alignment continuously (call in loop)
     */
    public void maintainAlignment() {
        if (currentState != AlignmentState.ALIGNED) return;

        // Try to get fresh target data
        if (!findTarget()) {
            currentState = AlignmentState.TARGET_LOST;
            return;
        }

        // Check if still aligned
        double absTx = Math.abs(currentTx);
        if (absTx > AlignmentParams.TX_TOLERANCE_DEGREES * 2) {
            // Lost alignment - restart
            consecutiveAlignedFrames = 0;
            currentState = AlignmentState.ALIGNING;

            // Update zone and gains
            currentZone = TXZone.getZone(absTx);
            pidController.setZoneGains(currentZone);

            // Calculate correction
            double positionChange = pidController.calculate(currentTx);
            double newTarget = turretController.getCurrentPosition() + positionChange;
            turretController.setTargetPosition(newTarget);
        }

        // Always update turret
        turretController.update();
    }

    /**
     * Execute search pattern
     */
    private void executeSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.stop();
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.stop();
            opMode.telemetry.addLine("⚠️ Search timeout");
            opMode.telemetry.update();
            return;
        }

        // Try to find target during search
        if (findTarget()) {
            // Found target! Start alignment
            currentState = AlignmentState.ALIGNING;

            double absTx = Math.abs(currentTx);
            currentZone = TXZone.getZone(absTx);
            pidController.setZoneGains(currentZone);

            try {
                executeAlignment();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
            return;
        }

        // Continue search - increment position periodically
        if (searchWaitTimer.milliseconds() > TargetSearch.SEARCH_WAIT_MS) {
            double currentPos = turretController.getCurrentPosition();
            double newTarget = currentPos + (searchDirection * TargetSearch.SEARCH_INCREMENT_DEGREES);

            // Check limits and reverse if needed
            if (TurretController.TURRET_ENABLE_LIMITS) {
                if (newTarget >= TurretController.TURRET_MAX_ANGLE) {
                    searchDirection = -1;
                    newTarget = currentPos - TargetSearch.SEARCH_INCREMENT_DEGREES;
                } else if (newTarget <= TurretController.TURRET_MIN_ANGLE) {
                    searchDirection = 1;
                    newTarget = currentPos + TargetSearch.SEARCH_INCREMENT_DEGREES;
                }
            }

            turretController.setTargetPosition(newTarget);
            searchWaitTimer.reset();

            opMode.telemetry.addLine("🔍 Searching...");
            opMode.telemetry.addData("Search Time", "%.1fs", searchTimer.seconds());
            opMode.telemetry.addData("Turret Pos", "%.1f°", currentPos);
            opMode.telemetry.update();
        }

        // Update turret
        turretController.update();
    }

    /**
     * Main update method - call this in your control loop
     */
    public void update() {
        if (!isActive) return;

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

    // === HELPER METHODS ===

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

            for (int i = 0; i < AlignmentParams.LIMELIGHT_SAMPLES; i++) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == targetTagId) {
                            sumTx += fiducial.getTargetXDegrees();
                            validReadings++;
                            break;
                        }
                    }
                }
            }

            if (validReadings > 0) {
                currentTx = sumTx / validReadings;
                hasTarget = true;
                return true;
            }

        } catch (Exception ignored) {
        }

        hasTarget = false;
        return false;
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Target_Tag_ID", targetTagId);

        // Zone and PID parameters
        packet.put("Zone", currentZone.getDescription());
        packet.put("PID_KP", pidController.getKP());
        packet.put("PID_KD", pidController.getKD());

        // Errors
        packet.put("TX_Error", currentTx);
        packet.put("Abs_TX_Error", Math.abs(currentTx));

        // Turret position
        packet.put("Turret_Position", turretController.getCurrentPosition());
        packet.put("Turret_Target", turretController.getTargetPosition());
        packet.put("Turret_Moving", turretController.isMoving());

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

        opMode.telemetry.addLine(">>> TX ERROR <<<");
        opMode.telemetry.addData("TX", "%.2f°", currentTx);
        opMode.telemetry.addData("Abs TX", "%.2f°", Math.abs(currentTx));
        opMode.telemetry.addData("Zone", currentZone.getDescription());
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> PID GAINS <<<");
        opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
        opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> TURRET <<<");
        opMode.telemetry.addData("Position", "%.1f°", turretController.getCurrentPosition());
        opMode.telemetry.addData("Target", "%.1f°", turretController.getTargetPosition());
        opMode.telemetry.addData("Moving", turretController.isMoving());
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Has Target", hasTarget);

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

    public double getTxError() {
        return currentTx;
    }

    public double getAbsTxError() {
        return Math.abs(currentTx);
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public String getCurrentZone() {
        return currentZone.getDescription();
    }

    public int getTargetTagId() {
        return targetTagId;
    }
}