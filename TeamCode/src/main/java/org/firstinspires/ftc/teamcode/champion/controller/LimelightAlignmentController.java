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
 * Odometry-based AprilTag alignment with Zone-Based PID control
 * Selects PID constants based on initial TX angle zones
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double HEADING_TOLERANCE_DEGREES = 1.5;
        public static long ALIGNMENT_TIMEOUT_MS = 1000;
        public static int ALIGNED_FRAMES_REQUIRED = 5;
        public static double SETTLING_TIME_MS = 100;
    }

    @Config
    public static class PIDParams {
        // Zone 1: 0-5 degrees
        public static double ZONE1_KP = 8.5;
        public static double ZONE1_KD = 0.0;

        // Zone 2: 5-10 degrees
        public static double ZONE2_KP = 7.5;
        public static double ZONE2_KD = 0.0;

        // Zone 3: >10 degrees
        public static double ZONE3_KP = 6;
        public static double ZONE3_KD = 0.0;

        // Shared parameters
        public static double KI = 0.0;  // Not used

        // Output limits (degrees/sec) - shared across all zones
        public static double MAX_ANGULAR_VELOCITY = 100.0;
        public static double MIN_ANGULAR_VELOCITY = 10.0;

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

        public void setTarget(double targetHeading) {
            this.setpoint = targetHeading;
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

        public double calculate(double currentHeading) {
            // Calculate error (shortest path)
            double error = angleDifferenceDegrees(currentHeading, setpoint);

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
            output = Math.max(-PIDParams.MAX_ANGULAR_VELOCITY,
                    Math.min(PIDParams.MAX_ANGULAR_VELOCITY, output));

            // Apply minimum speed if we're still moving
            if (Math.abs(output) > 0 && Math.abs(output) < PIDParams.MIN_ANGULAR_VELOCITY) {
                output = Math.copySign(PIDParams.MIN_ANGULAR_VELOCITY, output);
            }

            // Store for next iteration
            lastError = error;

            return output;
        }

        public double getError(double currentHeading) {
            return angleDifferenceDegrees(currentHeading, setpoint);
        }

        public boolean isAtSetpoint(double currentHeading) {
            return Math.abs(getError(currentHeading)) <= AlignmentParams.HEADING_TOLERANCE_DEGREES;
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
    private double initialHeadingDegrees = 0;
    private double targetHeadingDegrees = 0;
    private boolean targetHeadingCalculated = false;

    // Zone tracking
    private TXZone currentZone = TXZone.ZONE2_MEDIUM;

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();

    // === INITIALIZATION ===

    public LimelightAlignmentController(LinearOpMode opMode, SixWheelDriveController driveController) throws Exception {
        this.opMode = opMode;
        this.driveController = driveController;

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
        targetHeadingCalculated = false;
        alignmentTimer.reset();

        // Update odometry and get initial heading
        driveController.updateOdometry();
        initialHeadingDegrees = driveController.getHeadingDegrees();

        // Try to find target
        if (findTarget()) {
            // Determine zone based on absolute initial TX
            double absTx = Math.abs(initialTx);
            currentZone = TXZone.getZone(absTx);

            // Set PID gains based on zone
            pidController.setZoneGains(currentZone);

            // Calculate target heading from initial tx
            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;

            // Initialize PID controller with target
            pidController.setTarget(targetHeadingDegrees);

            opMode.telemetry.addLine("=== ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial TX", "%.2f°", initialTx);
            opMode.telemetry.addData("Abs TX", "%.2f°", absTx);
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addData("Initial Heading", "%.2f°", initialHeadingDegrees);
            opMode.telemetry.addData("Target Heading", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.update();

            // Execute PID-controlled turn
            try {
                turnToHeadingPID(targetHeadingDegrees);
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
     * Turn to target heading using PID control with zone-based gains
     * This is a BLOCKING method that returns when aligned or timeout
     * @noinspection BusyWait
     */
    private void turnToHeadingPID(double targetHeadingDegrees) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        int stableFrames = 0;

        // PID gains are already set based on zone - they won't change during this turn
        pidController.setTarget(targetHeadingDegrees);

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
                opMode.telemetry.addLine("⚠️ Alignment timeout");
                opMode.telemetry.update();
                break;
            }

            // Update odometry - read IMU heading
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();

            // Calculate PID output (angular velocity in degrees/sec)
            double angularVelocity = pidController.calculate(currentHeading);

            // Get current error for display
            double headingError = pidController.getError(currentHeading);

            // Display status
            opMode.telemetry.addLine("=== PID ALIGNMENT ===");
            opMode.telemetry.addData("Zone", currentZone.getDescription());
            opMode.telemetry.addData("KP", "%.2f", pidController.getKP());
            opMode.telemetry.addData("KD", "%.2f", pidController.getKD());
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Current", "%.2f°", currentHeading);
            opMode.telemetry.addData("Target", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.addData("Error", "%.2f°", headingError);
            opMode.telemetry.addData("Angular Vel", "%.1f°/s", angularVelocity);
            opMode.telemetry.addData("Stable", "%d/%d", stableFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);

            // Check if aligned
            if (pidController.isAtSetpoint(currentHeading)) {
                stableFrames++;
                opMode.telemetry.addLine("✓ Within tolerance");

                if (stableFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    driveController.stopDrive();
                    sleep((long)AlignmentParams.SETTLING_TIME_MS);
                    break;
                }

                // Stop while counting stable frames
                driveController.stopDrive();
            } else {
                // Reset stable counter if we drift
                stableFrames = 0;

                // Apply the PID-calculated angular velocity
                driveController.setAngularVelocity(angularVelocity);
            }

            opMode.telemetry.update();

            // Loop timing (50Hz)
            sleep(20);
        }

        // Ensure stopped
        driveController.stopDrive();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        pidController.reset();

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        driveController.stopDrive();
    }

    public void maintainAlignment() {
        if (!targetHeadingCalculated) return;

        // Continuously apply PID to maintain heading (uses same zone gains)
        driveController.updateOdometry();
        double currentHeading = driveController.getHeadingDegrees();

        // Use PID to calculate correction
        double angularVelocity = pidController.calculate(currentHeading);

        // Apply correction if needed
        if (Math.abs(angularVelocity) > PIDParams.DEADBAND_DEGREES) {
            driveController.setAngularVelocity(angularVelocity);
        } else {
            driveController.stopDrive();
        }
    }

    // === HELPER METHODS ===

    /**
     * Normalize angle to [-180, 180] range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
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
            driveController.stopDrive();
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            driveController.stopDrive();
            return;
        }

        // Try to find target during search
        if (findTarget()) {
            // Found target! Determine zone and set gains
            driveController.updateOdometry();
            initialHeadingDegrees = driveController.getHeadingDegrees();

            double absTx = Math.abs(initialTx);
            currentZone = TXZone.getZone(absTx);
            pidController.setZoneGains(currentZone);

            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;
            hasTarget = true;

            try {
                turnToHeadingPID(targetHeadingDegrees);
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
            return;
        }

        // Continue searching - slow rotation
        driveController.tankDrive(-TargetSearch.SEARCH_SPEED, TargetSearch.SEARCH_SPEED);
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
        packet.put("Target_Calculated", targetHeadingCalculated);

        // Zone and PID parameters
        packet.put("Zone", currentZone.getDescription());
        packet.put("PID_KP", pidController.getKP());
        packet.put("PID_KD", pidController.getKD());

        // Headings
        driveController.updateOdometry();
        double currentHeading = driveController.getHeadingDegrees();
        packet.put("Initial_Heading", initialHeadingDegrees);
        packet.put("Current_Heading", currentHeading);
        packet.put("Target_Heading", targetHeadingDegrees);
        packet.put("Initial_TX", initialTx);
        packet.put("Abs_TX", Math.abs(initialTx));

        if (targetHeadingCalculated) {
            double error = angleDifferenceDegrees(currentHeading, targetHeadingDegrees);
            packet.put("Heading_Error", error);
        }

        // Velocities
        packet.put("Left_Wheel_Vel", driveController.getLeftVelocity());
        packet.put("Right_Wheel_Vel", driveController.getRightVelocity());

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
        opMode.telemetry.addLine("  ZONE-BASED ALIGNMENT");
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
        if (!targetHeadingCalculated) return 999;
        driveController.updateOdometry();
        return Math.abs(angleDifferenceDegrees(driveController.getHeadingDegrees(), targetHeadingDegrees));
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public String getCurrentZone() {
        return currentZone.getDescription();
    }

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════╗");
        opMode.telemetry.addLine("║ ZONE-BASED ALIGNMENT  ║");
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