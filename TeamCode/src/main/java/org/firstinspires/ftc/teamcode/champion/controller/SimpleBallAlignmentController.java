package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Ball Alignment Controller - Handles invalid results and uses Python-calculated TX
 */
@Config
public class SimpleBallAlignmentController {
    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;
    @Config
    public static class BallAlignPID {
        public static double KP = 0.07;
        public static double KI = 0.0;
        public static double KD = 0.015;
        public static double INTEGRAL_LIMIT = 3.0;
    }

    @Config
    public static class BallAlignZones {
        public static double TOLERANCE = 0.5;    // degrees
        public static double DEAD_ZONE = 2.0;    // degrees
    }

    @Config
    public static class BallAlignSpeed {
        public static double MAX_SPEED = 0.5;
        public static double MIN_SPEED = 0.0;
        public static double MAX_ACCELERATION = 1.5;
    }

    @Config
    public static class BallAlignFiltering {
        public static double INPUT_FILTER = 0.7;
        public static double OUTPUT_FILTER = 0.8;
    }

    @Config
    public static class BallAlignAdvanced {
        public static boolean ADAPTIVE_GAIN = true;
        public static double ADAPTIVE_FACTOR = 0.8;
        public static int ALIGNED_FRAMES = 10;
        public static double PROGRESSIVE_POWER = 1.5;
    }

    @Config
    public static class BallAlignSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_SPEED = 0.4;
        public static double SEARCH_DELAY_MS = 300;
        public static double MAX_SEARCH_TIME_MS = 8000;
    }

    @Config
    public static class BallDetection {
        public static double MIN_BALL_AREA = 300.0;
        public static int TARGET_COLOR = 0;          // 0=any, 1=green, 2=purple
        public static double MAX_TX_DEGREES = 30.0;
    }

    // Ball data from Python output (new format with TX/TY in array)
    // Supports multiple balls - Python returns closest/primary ball
    private double ballTx = 0;      // TX from Python (index 1)
    private double ballTy = 0;      // TY from Python (index 2)
    private double ballPixelX = 0;  // Center X from Python (index 3)
    private double ballPixelY = 0;  // Center Y from Python (index 4)
    private double ballDistance = 0;
    private int ballColor = 0;
    private double ballArea = 0;

    // Tracking state
    private double tx = 0;           // Current TX being used for control
    private double filteredTx = 0;   // Filtered TX for smooth control
    private boolean hasTarget = false;

    // PID state
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // Smoothing state
    private double lastOutput = 0;
    private double currentOutput = 0;
    private final ElapsedTime accelerationTimer = new ElapsedTime();

    // State tracking
    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }
    private AlignmentState currentState = AlignmentState.STOPPED;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();

    // Result tracking
    private final ElapsedTime initTimer = new ElapsedTime();
    private int invalidResultCount = 0;
    private int validResultCount = 0;
    private boolean hasSeenValidResult = false;

    // Control flags
    private boolean isActive = false;

    public SimpleBallAlignmentController(LinearOpMode opMode) throws Exception {
        this.opMode = opMode;

        try {
            this.driveController = new SixWheelDriveController(opMode);
        } catch (Exception e) {
            throw new Exception("Failed to initialize drive controller: " + e.getMessage());
        }

        this.dashboard = FtcDashboard.getInstance();

        try {
            // Get Limelight
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

            // Set pipeline BEFORE starting
            this.limelight.pipelineSwitch(0);  // Pipeline 0 for Python snapscript
            Thread.sleep(500);

            // Start Limelight
            this.limelight.start();

            // Wait for initialization
            Thread.sleep(1500);  // Give more time for pipeline to load

            opMode.telemetry.addLine("✓ Limelight initialized");
            opMode.telemetry.addLine("  Pipeline: 0 (Python)");
            opMode.telemetry.update();

        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }

        pidTimer.reset();
        accelerationTimer.reset();
        stateTimer.reset();
        searchTimer.reset();
        initTimer.reset();
    }

    public void startTracking() {
        isActive = true;

        // Try to get initial state
        updateTracking();

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
        }

        consecutiveAlignedFrames = 0;
        resetPID();
    }

    public void stopTracking() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        driveController.stopDrive();
        resetPID();
    }

    /**
     * Main alignment loop
     */
    public void align() {
        if (!isActive) return;

        updateTracking();

        // Handle states
        switch (currentState) {
            case SEARCHING:
                executeTargetSearch();
                break;
            case ALIGNING:
                executeSmoothedAlignment();
                break;
            case ALIGNED:
                maintainAlignment();
                break;
            case TARGET_LOST:
                handleTargetLost();
                break;
            case STOPPED:
                break;
        }

        sendDashboardData();
    }

    private void executeTargetSearch() {
        if (!BallAlignSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            smoothStop();
            return;
        }

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            resetPID();
            return;
        }

        if (searchTimer.milliseconds() > BallAlignSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            smoothStop();
            return;
        }

        double searchPower = BallAlignSearch.SEARCH_SPEED;
        driveController.tankDrive(-searchPower, searchPower);  // CCW
        currentOutput = searchPower;
        lastOutput = currentOutput;
    }

    private void executeSmoothedAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            return;
        }

        double headingError = filteredTx;
        double absError = Math.abs(headingError);

        if (absError <= BallAlignZones.TOLERANCE) {
            consecutiveAlignedFrames++;
            if (consecutiveAlignedFrames >= BallAlignAdvanced.ALIGNED_FRAMES) {
                currentState = AlignmentState.ALIGNED;
                stateTimer.reset();
                smoothStop();
                return;
            }
        } else {
            consecutiveAlignedFrames = Math.max(0, consecutiveAlignedFrames - 2);
        }

        double turnPower = calculateSmoothedPID(headingError);

        if (absError < BallAlignZones.DEAD_ZONE) {
            double scale = absError / BallAlignZones.DEAD_ZONE;
            scale = Math.pow(scale, BallAlignAdvanced.PROGRESSIVE_POWER);
            turnPower *= scale;
        }

        turnPower = applyAccelerationLimit(turnPower);

        currentOutput = (BallAlignFiltering.OUTPUT_FILTER * turnPower) +
                ((1 - BallAlignFiltering.OUTPUT_FILTER) * lastOutput);
        lastOutput = currentOutput;

        applyTurn(currentOutput);
    }

    private double calculateSmoothedPID(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return BallAlignPID.KP * error;
        }

        lastTime = currentTime;

        double gainMultiplier = 1.0;
        if (BallAlignAdvanced.ADAPTIVE_GAIN) {
            double absError = Math.abs(error);
            if (absError < BallAlignZones.DEAD_ZONE * 2) {
                gainMultiplier = BallAlignAdvanced.ADAPTIVE_FACTOR +
                        (1 - BallAlignAdvanced.ADAPTIVE_FACTOR) * (absError / (BallAlignZones.DEAD_ZONE * 2));
            }
        }

        double pTerm = BallAlignPID.KP * error * gainMultiplier;

        if (Math.signum(error) != Math.signum(lastError) && Math.abs(error) > BallAlignZones.TOLERANCE) {
            integralSum *= 0.5;
        } else {
            integralSum += error * dt;
        }
        integralSum = Range.clip(integralSum, -BallAlignPID.INTEGRAL_LIMIT, BallAlignPID.INTEGRAL_LIMIT);
        double iTerm = BallAlignPID.KI * integralSum;

        double derivative = (error - lastError) / dt;
        derivative = Range.clip(derivative, -50, 50);
        double dTerm = BallAlignPID.KD * derivative * gainMultiplier;
        lastError = error;

        double output = pTerm + iTerm + dTerm;
        output = Range.clip(output, -BallAlignSpeed.MAX_SPEED, BallAlignSpeed.MAX_SPEED);

        if (Math.abs(error) > BallAlignZones.DEAD_ZONE &&
                Math.abs(output) > 0 &&
                Math.abs(output) < BallAlignSpeed.MIN_SPEED &&
                BallAlignSpeed.MIN_SPEED > 0) {
            output = Math.signum(output) * BallAlignSpeed.MIN_SPEED;
        }

        return output;
    }

    private double applyAccelerationLimit(double targetPower) {
        double dt = accelerationTimer.seconds();
        accelerationTimer.reset();

        if (dt <= 0 || dt > 0.5) return targetPower;

        double maxChange = BallAlignSpeed.MAX_ACCELERATION * dt;
        double powerDiff = targetPower - lastOutput;

        if (Math.abs(powerDiff) > maxChange) {
            return lastOutput + Math.signum(powerDiff) * maxChange;
        }

        return targetPower;
    }

    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        double absError = Math.abs(filteredTx);

        if (absError > BallAlignZones.DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            return;
        }

        if (absError > BallAlignZones.TOLERANCE) {
            double tinyCorrection = BallAlignPID.KP * filteredTx * 0.3;
            tinyCorrection = Range.clip(tinyCorrection, -0.05, 0.05);
            currentOutput = (0.9 * tinyCorrection) + (0.1 * lastOutput);
            lastOutput = currentOutput;
            applyTurn(currentOutput);
        } else {
            smoothStop();
        }
    }

    private void handleTargetLost() {
        if (BallAlignSearch.ENABLE_SEARCH && stateTimer.milliseconds() > BallAlignSearch.SEARCH_DELAY_MS) {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            return;
        }

        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
        }
    }

    /**
     * Update tracking - IGNORES isValid() for Python pipelines
     */
    private void updateTracking() {
        if (limelight == null) {
            hasTarget = false;
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();

            // Handle null result
            if (result == null) {
                hasTarget = false;
                return;
            }

            // IMPORTANT: For Python pipelines, isValid() checks for standard targets (AprilTags, etc.)
            // We IGNORE isValid() and only check the Python output directly!
            // The Python script's first element (pythonOutput[0]) tells us if a ball was found

            // Get Python output - this is what matters for custom detection
            double[] pythonOutput = result.getPythonOutput();

            if (pythonOutput != null && pythonOutput.length >= 8) {
                // SUCCESS! We have Python data
                validResultCount++;
                hasSeenValidResult = true;

                // Python output format (from your updated script):
                // [0]: target_found (1/0) - THIS IS OUR VALIDITY CHECK!
                // [1]: tx (degrees) - CALCULATED IN PYTHON
                // [2]: ty (degrees) - CALCULATED IN PYTHON
                // [3]: center_x (pixels)
                // [4]: center_y (pixels)
                // [5]: distance (inches)
                // [6]: ball_color (1=green, 2=purple)
                // [7]: area (pixels²)

                // Use Python's found flag as our validity indicator
                boolean pythonFoundBall = pythonOutput[0] > 0.5;

                if (pythonFoundBall) {
                    boolean previousHasTarget = hasTarget;

                    // Get TX directly from Python (already calculated in degrees!)
                    ballTx = pythonOutput[1];
                    ballTy = pythonOutput[2];
                    ballPixelX = pythonOutput[3];
                    ballPixelY = pythonOutput[4];
                    ballDistance = pythonOutput[5];
                    ballColor = (int)pythonOutput[6];
                    ballArea = pythonOutput[7];

                    // Validate the ball
                    if (ballArea >= BallDetection.MIN_BALL_AREA &&
                            Math.abs(ballTx) <= BallDetection.MAX_TX_DEGREES) {

                        // Check color filter if enabled
                        if (BallDetection.TARGET_COLOR == 0 ||
                                BallDetection.TARGET_COLOR == ballColor) {

                            hasTarget = true;
                            tx = ballTx;  // Use Python-calculated TX directly

                            // Apply filtering
                            if (!previousHasTarget) {
                                filteredTx = tx;
                            } else {
                                filteredTx = (BallAlignFiltering.INPUT_FILTER * tx) +
                                        ((1 - BallAlignFiltering.INPUT_FILTER) * filteredTx);
                            }
                        } else {
                            hasTarget = false;
                        }
                    } else {
                        hasTarget = false;
                    }
                } else {
                    // Python says no ball found
                    hasTarget = false;
                }
            } else {
                // No Python output or wrong length - THIS is the actual problem
                hasTarget = false;
                invalidResultCount++;

                // Debug logging
                if (pythonOutput == null) {
                    if (initTimer.seconds() > 3) {  // Only log after init time
                        opMode.telemetry.addData("Warning", "Python output NULL - check pipeline");
                    }
                } else {
                    opMode.telemetry.addData("Warning", "Python array length: " + pythonOutput.length + " (expected 8)");
                }
            }
        } catch (Exception e) {
            hasTarget = false;
            opMode.telemetry.addData("Limelight Error", e.getMessage());
        }
    }

    private void smoothStop() {
        currentOutput *= 0.8;
        if (Math.abs(currentOutput) < 0.02) {
            currentOutput = 0;
            driveController.stopDrive();
        } else {
            applyTurn(currentOutput);
        }
        lastOutput = currentOutput;
    }

    private void applyTurn(double turnPower) {
        if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < 0.05) {
            turnPower = 0;
        }
        driveController.tankDrive(turnPower, -turnPower);
    }

    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        lastOutput = 0;
        currentOutput = 0;
        pidTimer.reset();
        accelerationTimer.reset();
    }

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Ball_State", currentState.toString());
        packet.put("Has_Ball", hasTarget);
        packet.put("TX_Python", ballTx);  // TX from Python calculation
        packet.put("TY_Python", ballTy);
        packet.put("TX_Filtered", filteredTx);
        packet.put("Error_Abs", Math.abs(filteredTx));
        packet.put("Ball_Color", ballColor == 1 ? "GREEN" : ballColor == 2 ? "PURPLE" : "NONE");
        packet.put("Ball_PixelX", ballPixelX);
        packet.put("Ball_Area", ballArea);
        packet.put("Distance", ballDistance);
        packet.put("Output", currentOutput);
        packet.put("Aligned_Frames", consecutiveAlignedFrames);
        packet.put("Valid_Results", validResultCount);
        packet.put("Invalid_Results", invalidResultCount);

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("╔═══ BALL ALIGNMENT ═══╗");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Ball", hasTarget ? "FOUND" : "LOST");

        // Show result statistics
        if (!hasSeenValidResult && initTimer.seconds() > 3) {
            opMode.telemetry.addLine("⚠️ No valid results yet!");
            opMode.telemetry.addData("Invalid Count", invalidResultCount);
        }

        if (hasTarget) {
            String color = ballColor == 1 ? "GREEN" : ballColor == 2 ? "PURPLE" : "UNKNOWN";
            opMode.telemetry.addData("Color", color);
            opMode.telemetry.addData("TX", "%.2f°", tx);
            opMode.telemetry.addData("TX Filtered", "%.2f°", filteredTx);
            opMode.telemetry.addData("Area", "%.0f px", ballArea);
            opMode.telemetry.addData("Distance", "%.1f in", ballDistance);
            opMode.telemetry.addData("Output", "%.3f", currentOutput);

            if (currentState == AlignmentState.ALIGNED) {
                opMode.telemetry.addLine("✅ ALIGNED!");
            }
        }

        if (currentState == AlignmentState.SEARCHING) {
            opMode.telemetry.addData("Search Time", "%.1fs", searchTimer.seconds());
        }
    }

    // Getters
    public boolean isTracking() { return isActive; }
    public boolean isAligned() { return currentState == AlignmentState.ALIGNED; }
    public boolean hasBall() { return hasTarget; }
    public AlignmentState getState() { return currentState; }
    public double getTargetError() { return Math.abs(filteredTx); }
    public double getTx() { return tx; }
    public double getBallDistance() { return ballDistance; }
    public int getBallColor() { return ballColor; }
    public double getBallPixelX() { return ballPixelX; }
    public double getBallArea() { return ballArea; }
    public double getMotorOutput() { return currentOutput; }
    public boolean hasSeenValidResult() { return hasSeenValidResult; }
}