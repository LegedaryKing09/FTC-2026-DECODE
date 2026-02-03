package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;

import java.util.Locale;

/**
 * 超级无敌高级Teleop
 *
 * FIELD-CENTRIC AUTO-AIM:
 * - Inherits position from auton via PoseStorage
 * - Continuously tracks robot position on field
 * - Auto-calculates turret angle to aim at TARGET (0,0) or custom position
 * - Pressing Y or A preset calculates angle based on current robot position
 *
 * === DRIVER 1 (gamepad1) - ARCADE DRIVE (FULL POWER) ===
 * Left Stick Y:   Forward/Backward (1.0 power)
 * Right Stick X:  Turn Left/Right (1.0 power)
 * Right Bumper:   Toggle intake + transfer + uptake (all three)
 *                 Ball switch auto-stops uptake when ball detected,
 *                 auto-restarts when ball leaves
 * Left Bumper:    HOLD to VOMIT (reverse intake + transfer + uptake)
 *
 * === DRIVER 2 (gamepad2) ===
 * Right Trigger:  HOLD to SHOOT - runs intake + transfer + uptake,
 *                 BYPASSES ball detection (forces balls through shooter)
 *
 * Left Bumper:    RETRACT ramp (angle goes more negative)
 * Left Trigger:   EXTEND ramp (angle goes less negative, toward 0°)
 *
 * D-Pad Up:       RPM +100
 * D-Pad Down:     RPM -100
 * D-Pad Right:    Turret target +1 degree (when auto-aim enabled)
 * D-Pad Left:     Turret target -1 degree (when auto-aim enabled)
 *
 * Right Stick X:  Manual turret - DISABLES AUTO-AIM when moved!
 *                 Auto-aim stays off until next preset button is pressed
 *
 * Y Button:       FAR preset - calculates turret angle to (0,0) based on position
 * A Button:       CLOSE preset - AUTO RPM/RAMP based on distance to goal
 * X Button:       Set RPM to IDLE (2000 RPM)
 */
@Config
@TeleOp(name = "Undefeated Super Powerful Teleop Blue", group = "Competition")
public class MyOnlyTeleop1 extends LinearOpMode {

    // === PRESETS ===
    public static double FAR_RPM = 4600.0;
    public static double FAR_RAMP_ANGLE = 0.6;
    public static double FAR_TURRET = -23;
    // FAR_TURRET_ANGLE removed - now calculated dynamically

    // CLOSE preset now uses distance-based lookup
    public static double CLOSE_TURRET = -45;

    public static double IDLE_RPM = 2000.0;

    // === DISTANCE-BASED SHOOTING TABLE ===
    // Distance (inches) = base distance + 18 inches offset
    // Each entry: {distance, ramp angle, RPM}
    // Piecewise: if distance is within ±6 inches of a calibrated point, use that setting
    public static double DISTANCE_TOLERANCE = 6.0;  // ±6 inches

    // Calibration points (distance already includes +18 offset)
    public static double DIST_1 = 24.0;   public static double RAMP_1 = 0.21;  public static double RPM_1 = 2750.0;
    public static double DIST_2 = 36.0;   public static double RAMP_2 = 0.34;  public static double RPM_2 = 3100.0;
    public static double DIST_3 = 48.0;   public static double RAMP_3 = 0.36;  public static double RPM_3 = 3150.0;
    public static double DIST_4 = 60.0;   public static double RAMP_4 = 0.40;  public static double RPM_4 = 3250.0;
    public static double DIST_5 = 72.0;   public static double RAMP_5 = 0.43;  public static double RPM_5 = 3450.0;

    // === TARGET POSITION (where turret aims at) ===
    public static double TARGET_X = 0.0;  // Field X coordinate to aim at
    public static double TARGET_Y = 0.0;  // Field Y coordinate to aim at

    // === AUTON STARTING POSITION ===
    public static double AUTON_START_X = 48;
    public static double AUTON_START_Y = 137;
    public static double AUTON_START_HEADING = 0.0;  // degrees

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = 0.01;  // Always positive, direction handled in code
    public static double TURRET_TARGET_INCREMENT = 1.0;
    public static double RPM_INCREMENT = 50.0;
    public static double TURRET_MANUAL_SENSITIVITY = 0.2;

    // === BALL DETECTION ===
    // Switch reads 3.3V when not pressed, 0V when pressed (ball detected)
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // === CONTROLLERS ===
    private SixWheelDriveController drive;
    private TurretController turret;
    private TurretFieldController turretField;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Ball detection switch
    private AnalogInput uptakeSwitch;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // === GAMEPAD 1 BUTTON STATES ===
    private boolean lastRB1 = false;

    // === GAMEPAD 2 BUTTON STATES ===
    private boolean lastY2 = false;
    private boolean lastA2 = false;
    private boolean lastX2 = false;
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastLB2 = false;
    private boolean lastLT2Pressed = false;

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.2;

    // === STATE TRACKING ===
    private boolean intakeModeActive = false;
    private boolean allSystemsFromTrigger = false;
    private double currentTargetRPM = 0;

    // Debug
    private String rampDebug = "";

    // === Preset mode tracking ===
    private enum PresetMode { IDLE, FAR, CLOSE }
    private PresetMode currentPresetMode = PresetMode.IDLE;

    // === Auto-aim state ===
    private boolean autoAimEnabled = false;
    public double turretError;
    private double turretOffset = 0.0;  // Persistent offset from D-pad adjustments

    // === Distance tracking for telemetry ===
    private double lastCalculatedDistance = 0;
    private String lastDistanceMatch = "NONE";

    @Override
    public void runOpMode() {
        // Read the static pose from auton
        Pose2d autonPose = PoseStorage.currentPose;

        initializeHardware();

        sleep(300);

        // Set position from auton, or use AUTON_START values if auton was skipped (pose is 0,0,0)
        if (drive != null) {
            double startX, startY, startHeading;

            // Check if auton was skipped (PoseStorage is at origin)
            if (autonPose.position.x == 0 && autonPose.position.y == 0 && autonPose.heading.toDouble() == 0) {
                // Auton skipped - use configured start position
                startX = AUTON_START_X;
                startY = AUTON_START_Y;
                startHeading = Math.toRadians(AUTON_START_HEADING);
            } else {
                // Auton ran - use saved pose
                startX = autonPose.position.x;
                startY = autonPose.position.y;
                startHeading = autonPose.heading.toDouble();
            }

            drive.setPosition(startX, startY, startHeading);
            drive.updateOdometry();
        }

        waitForStart();
        runtime.reset();

        // Initialize turret angle tracking AFTER start
        if (turret != null) {
            turret.restoreAngle(PoseStorage.turretAngle);
        }

        // Initialize ramp - sets current position as 0°
        if (ramp != null) {
            ramp.initialize();
        }

        // Auto-aim starts DISABLED
        if (turretField != null) {
            turretField.disable();
        }

        // Start shooter at idle RPM (2000)
        if (shooter != null) {
            shooter.setTargetRPM(IDLE_RPM);
            currentTargetRPM = IDLE_RPM;
            shooter.startShooting();
        }

        loopTimer.reset();

        while (opModeIsActive()) {

            // === FAST UPDATES (every loop) ===
            handleDriveControls();
            handleDriver1Controls();
            handleDriver2Controls();

            // === BALL DETECTION: Auto-control uptake based on switch ===
            // Switch reads 3.3V when not pressed, 0V when pressed (ball detected)
            // BYPASS: When gamepad2 RT is held, ignore ball detection and keep uptake running
            boolean bypassBallDetection = gamepad2.right_trigger > TRIGGER_THRESHOLD;

            if (intakeModeActive && uptakeSwitch != null && !bypassBallDetection) {
                double switchVoltage = uptakeSwitch.getVoltage();
                if (switchVoltage < UPTAKE_SWITCH_THRESHOLD) {
                    // Ball detected - stop uptake (intake + transfer keep running)
                    if (uptake != null && uptake.isActive()) {
                        uptake.toggle();
                    }
                } else {
                    // No ball - restart uptake if it's not running
                    if (uptake != null && !uptake.isActive()) {
                        uptake.reversed = false;
                        uptake.toggle();
                    }
                }
            }

            // Update all controllers
            updateAllSystems();

            // === TELEMETRY ===
            updateTelemetry();
        }

        // Cleanup
        if (shooter != null) shooter.stopShooting();
        if (turretField != null) turretField.disable();
    }

    /**
     * Get RPM and Ramp angle based on distance using piecewise function
     * Returns double[2] where [0] = RPM, [1] = ramp angle
     * Distance tolerance is ±6 inches from calibrated points
     */
    private double[] getShootingParamsForDistance(double distance) {
        // Check each calibration point with ±6 inch tolerance
        // Points are checked in order from closest to farthest

        if (Math.abs(distance - DIST_1) <= DISTANCE_TOLERANCE) {
            lastDistanceMatch = String.format(Locale.US, "%.0f in (±6)", DIST_1);
            return new double[]{RPM_1, RAMP_1};
        }
        if (Math.abs(distance - DIST_2) <= DISTANCE_TOLERANCE) {
            lastDistanceMatch = String.format(Locale.US, "%.0f in (±6)", DIST_2);
            return new double[]{RPM_2, RAMP_2};
        }
        if (Math.abs(distance - DIST_3) <= DISTANCE_TOLERANCE) {
            lastDistanceMatch = String.format(Locale.US, "%.0f in (±6)", DIST_3);
            return new double[]{RPM_3, RAMP_3};
        }
        if (Math.abs(distance - DIST_4) <= DISTANCE_TOLERANCE) {
            lastDistanceMatch = String.format(Locale.US, "%.0f in (±6)", DIST_4);
            return new double[]{RPM_4, RAMP_4};
        }
        if (Math.abs(distance - DIST_5) <= DISTANCE_TOLERANCE) {
            lastDistanceMatch = String.format(Locale.US, "%.0f in (±6)", DIST_5);
            return new double[]{RPM_5, RAMP_5};
        }

        // No match found - use closest calibration point
        double minDiff = Double.MAX_VALUE;
        double[] bestParams = new double[]{RPM_3, RAMP_3};  // Default to middle
        String bestMatch = "FALLBACK";

        double[] distances = {DIST_1, DIST_2, DIST_3, DIST_4, DIST_5};
        double[] rpms = {RPM_1, RPM_2, RPM_3, RPM_4, RPM_5};
        double[] ramps = {RAMP_1, RAMP_2, RAMP_3, RAMP_4, RAMP_5};

        for (int i = 0; i < distances.length; i++) {
            double diff = Math.abs(distance - distances[i]);
            if (diff < minDiff) {
                minDiff = diff;
                bestParams = new double[]{rpms[i], ramps[i]};
                bestMatch = String.format(Locale.US, "CLOSEST: %.0f in", distances[i]);
            }
        }

        lastDistanceMatch = bestMatch;
        return bestParams;
    }

    /**
     * Update telemetry with ramp, RPM, turret angle, and odometry
     */
    private void updateTelemetry() {
        telemetry.addLine("═══ SHOOTER ═══");
        if (shooter != null) {
            telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", currentTargetRPM));
            telemetry.addData("Current RPM", String.format(Locale.US, "%.0f", shooter.getRPM()));
        }

        telemetry.addLine("═══ RAMP ═══");
        if (ramp != null) {
            telemetry.addData("Ramp Angle", String.format(Locale.US, "%.3f", ramp.getTargetAngle()));
            telemetry.addData("Ramp Current", String.format(Locale.US, "%.3f", ramp.getCurrentAngle()));
        }

        telemetry.addLine("═══ TURRET ═══");
        if (turret != null) {
            telemetry.addData("Turret Angle", String.format(Locale.US, "%.1f°", turret.getTurretAngle()));
        }
        telemetry.addData("Turret Offset", String.format(Locale.US, "%.1f°", turretOffset));
        if (turretField != null && autoAimEnabled) {
            telemetry.addData("Field Target", String.format(Locale.US, "%.1f°", turretField.getTargetFieldAngle()));
            telemetry.addData("Turret Error", String.format(Locale.US, "%.1f°", turretError));
        }

        telemetry.addLine("═══ ODOMETRY ═══");
        if (drive != null) {
            telemetry.addData("X", String.format(Locale.US, "%.2f in", drive.getX()));
            telemetry.addData("Y", String.format(Locale.US, "%.2f in", drive.getY()));
            telemetry.addData("Heading", String.format(Locale.US, "%.1f°", drive.getHeadingDegrees()));
        }

        telemetry.addLine("═══ DISTANCE ═══");
        telemetry.addData("To Goal", String.format(Locale.US, "%.1f in", getDistanceToTarget()));
        telemetry.addData("Last Calc", String.format(Locale.US, "%.1f in", lastCalculatedDistance));
        telemetry.addData("Match", lastDistanceMatch);

        telemetry.addLine("═══ STATUS ═══");
        telemetry.addData("Preset", currentPresetMode.toString());
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");

        telemetry.update();
    }

    /**
     * Initialize hardware
     */
    private void initializeHardware() {
        // Drive controller
        try {
            drive = new SixWheelDriveController(this);
            drive.setDriveMode(SixWheelDriveController.DriveMode.POWER);
        } catch (Exception e) {
            // Drive init failed
        }

        // Turret
        try {
            turret = new TurretController(this);
            turretField = new TurretFieldController(turret);
        } catch (Exception e) {
            // Turret init failed
        }

        // Ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception e) {
            // Ramp init failed
        }

        // Intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            // Intake init failed
        }
        intake = new NewIntakeController(intakeMotor);

        // Transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            // Transfer init failed
        }
        transfer = new NewTransferController(transferMotor);

        // Uptake
        CRServo uptakeServo = null;
        CRServo uptakeServo2 = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "servo1");
            uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        } catch (Exception e) {
            // Uptake init failed
        }
        uptake = new UptakeController(uptakeServo, uptakeServo2);

        // Uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            // Uptake switch init failed
        }

        // Shooter
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            // Shooter init failed
        }
        shooter = new NewShooterController(shooterMotor1,shooterMotor2);
    }

    // === DRIVE SENSITIVITY ===
    public static double DRIVE_EXPONENT = 2.0;  // 1.0 = linear, 2.0 = squared, 3.0 = cubed
    public static double TURN_EXPONENT = 3.0;   // Higher = more precision at low speeds

    /**
     * Apply sensitivity curve to joystick input
     * Keeps the sign but applies power curve for more control at low speeds
     */
    private double applyCurve(double input, double exponent) {
        return Math.copySign(Math.pow(Math.abs(input), exponent), input);
    }

    /**
     * Drive controls - ARCADE DRIVE with SENSITIVITY CURVE
     * Left Stick Y = Forward/Backward
     * Right Stick X = Turn
     */
    private void handleDriveControls() {
        if (drive == null) return;

        // Update odometry
        drive.updateOdometry();

        // Get raw joystick inputs
        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        // Apply sensitivity curves
        double drivePower = applyCurve(rawDrive, DRIVE_EXPONENT);
        double turnPower = applyCurve(rawTurn, TURN_EXPONENT);

        // Calculate left and right powers
        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;

        // Use tank drive (power mode)
        drive.tankDrive(leftPower, rightPower);
    }

    /**
     * Driver 1 controls
     */
    private void handleDriver1Controls() {
        // Right bumper - toggle intake + transfer + uptake (all three)
        boolean currentRB1 = gamepad1.right_bumper;
        if (currentRB1 && !lastRB1) {
            if (!intakeModeActive) {
                startIntakeMode();
            } else {
                stopIntakeMode();
            }
        }
        lastRB1 = currentRB1;

        // Left bumper - VOMIT (hold to reverse intake + transfer + uptake)
        if (gamepad1.left_bumper) {
            // Hold LB = reverse all systems
            if (intake != null) {
                intake.reversed = true;
                intake.setState(true);
            }
            if (transfer != null) {
                transfer.reversed = true;
                transfer.setState(true);
            }
            if (uptake != null) {
                uptake.reversed = true;
                uptake.setState(true);
            }
        } else {
            // Release LB = stop vomit (only if we were vomiting)
            if (intake != null && intake.reversed) {
                intake.reversed = false;
                intake.setState(false);
            }
            if (transfer != null && transfer.reversed) {
                transfer.reversed = false;
                transfer.setState(false);
            }
            if (uptake != null && uptake.reversed) {
                uptake.reversed = false;
                uptake.setState(false);
            }
        }
    }

    /**
     * Start intake mode - intake + transfer + uptake (all three)
     */
    private void startIntakeMode() {
        intakeModeActive = true;

        if (intake != null && !intake.isActive()) {
            intake.reversed = false;
            intake.toggle();
        }
        if (transfer != null && !transfer.isActive()) {
            transfer.reversed = false;
            transfer.toggle();
        }
        if (uptake != null && !uptake.isActive()) {
            uptake.reversed = false;
            uptake.toggle();
        }
    }

    /**
     * Stop intake mode - intake + transfer + uptake (all three)
     * If uptake is already off (from switch), it stays off
     */
    private void stopIntakeMode() {
        intakeModeActive = false;
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    /**
     * Driver 2 controls
     */
    private void handleDriver2Controls() {

        // === RT: HOLD TO SHOOT - Run ITU and bypass ball detection ===
        // This forces balls through the shooter regardless of the uptake switch
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // Start all three if not already running from trigger
            if (!allSystemsFromTrigger) {
                if (intake != null && !intake.isActive()) {
                    intake.reversed = false;
                    intake.toggle();
                }
                if (transfer != null && !transfer.isActive()) {
                    transfer.reversed = false;
                    transfer.toggle();
                }
                if (uptake != null && !uptake.isActive()) {
                    uptake.reversed = false;
                    uptake.toggle();
                }
                allSystemsFromTrigger = true;
            }
        } else {
            // Stop all three when released (only if started by trigger)
            if (allSystemsFromTrigger) {
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
                allSystemsFromTrigger = false;
            }
        }

        // === LB: RETRACT RAMP (more negative angle, toward -245°) ===

        boolean ltPressed = gamepad2.left_bumper;
        if (ltPressed && !lastLT2Pressed) {
            if (ramp != null) {
                double newTarget = ramp.getTargetAngle() - RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
                rampDebug = "LB RETRACT: " + ramp.getTargetAngle();
            }
        }

        lastLT2Pressed = ltPressed;

        // === LT: EXTEND RAMP (less negative angle, toward 0°) ===
        boolean currentLB2 = gamepad2.left_trigger> TRIGGER_THRESHOLD;
        if (currentLB2 && !lastLB2) {
            if (ramp != null) {
                double newTarget = ramp.getTargetAngle() + RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
                rampDebug = "LT EXTEND: " + ramp.getTargetAngle();
            }
        }
        lastLB2 = currentLB2;

        // === DPAD UP: RPM +100 ===
        boolean currentDpadUp2 = gamepad2.dpad_up;
        if (currentDpadUp2 && !lastDpadUp2) {
            currentTargetRPM += RPM_INCREMENT;
            if (shooter != null) {
                shooter.setTargetRPM(currentTargetRPM);
            }
        }
        lastDpadUp2 = currentDpadUp2;

        // === DPAD DOWN: RPM -100 ===
        boolean currentDpadDown2 = gamepad2.dpad_down;
        if (currentDpadDown2 && !lastDpadDown2) {
            currentTargetRPM -= RPM_INCREMENT;
            if (currentTargetRPM < IDLE_RPM) currentTargetRPM = IDLE_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(currentTargetRPM);
            }
        }
        lastDpadDown2 = currentDpadDown2;

        // === DPAD RIGHT: TURRET OFFSET +1° (adds to all calculated angles) ===
        boolean currentDpadRight2 = gamepad2.dpad_right;
        if (currentDpadRight2 && !lastDpadRight2) {
            turretOffset += TURRET_TARGET_INCREMENT;
        }
        lastDpadRight2 = currentDpadRight2;

        // === DPAD LEFT: TURRET OFFSET -1° (subtracts from all calculated angles) ===
        boolean currentDpadLeft2 = gamepad2.dpad_left;
        if (currentDpadLeft2 && !lastDpadLeft2) {
            turretOffset -= TURRET_TARGET_INCREMENT;
        }
        lastDpadLeft2 = currentDpadLeft2;

        // === RIGHT STICK X: MANUAL TURRET ===
        double turretInput = gamepad2.right_stick_x;
        if (turret != null) {
            if (Math.abs(turretInput) > 0.1) {
                // Joystick moved - disable auto-aim and take manual control
                if (autoAimEnabled) {
                    autoAimEnabled = false;
                    if (turretField != null) turretField.disable();
                }
                turret.setPower(turretInput * TURRET_MANUAL_SENSITIVITY);
            } else {
                // Joystick released - stop turret only if not in auto-aim mode
                if (!autoAimEnabled) {
                    turret.stop();
                }
            }
        }

        // === Y: FAR PRESET (always activates) ===
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            // Activate FAR preset
            currentPresetMode = PresetMode.FAR;
            currentTargetRPM = FAR_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(FAR_RPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);
            if (turretField != null) {
                // Calculate angle to target based on current robot position
                double targetAngle = calculateAngleToTarget();
                turretField.setTargetFieldAngle(targetAngle);
                turretField.enable();
                autoAimEnabled = true;
            }
        }
        lastY2 = currentY2;

        // === A: CLOSE PRESET - DISTANCE-BASED RPM/RAMP ===
        // Every press recalculates distance and updates RPM/ramp
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            currentPresetMode = PresetMode.CLOSE;

            // Calculate distance to goal and get shooting parameters
            double distance = getDistanceToTarget();
            lastCalculatedDistance = distance;
            double[] params = getShootingParamsForDistance(distance);
            double autoRPM = params[0];
            double autoRamp = params[1];

            // Apply the calculated RPM and ramp
            currentTargetRPM = autoRPM;
            if (shooter != null) {
                shooter.setTargetRPM(autoRPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(autoRamp);

            // Enable auto-aim if not already enabled
            if (turretField != null && !autoAimEnabled) {
                turretField.enable();
                autoAimEnabled = true;
            }
        }
        lastA2 = currentA2;

        // === X: SET RPM TO IDLE (2000) ===
        boolean currentX2 = gamepad2.x;
        if (currentX2 && !lastX2) {
            currentPresetMode = PresetMode.IDLE;
            currentTargetRPM = IDLE_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(IDLE_RPM);
            }
        }
        lastX2 = currentX2;
    }

    /**
     * Calculate the turret angle needed to aim at target (0,0)
     * Returns the angle in degrees relative to robot's forward direction
     *
     * Convention: CCW = negative, CW = positive
     * Formula: -atan(x / y) where x,y are robot's position
     */
    private double calculateAngleToTarget() {
        double robotX = getCorrectedX();
        double robotY = getCorrectedY();

        // Avoid division by zero
        if (Math.abs(robotY) < 0.001) {
            // Robot is at y=0, turret should point straight left or right
            return robotX > 0 ? -90.0 : 90.0;
        }

        // Calculate angle: -atan(x/y)
        // Negative because CCW is negative in our turret convention
        double angleRad = -Math.atan(robotX / robotY);
        double angleDeg = Math.toDegrees(angleRad);

        return angleDeg;
    }

    /**
     * Get distance from robot to target (in inches)
     */
    private double getDistanceToTarget() {
        double robotX = getCorrectedX();
        double robotY = getCorrectedY();

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Get corrected X position (now directly from drive since transformation is in controller)
     */
    private double getCorrectedX() {
        if (drive == null) return AUTON_START_X;
        return drive.getX();
    }

    /**
     * Get corrected Y position (now directly from drive since transformation is in controller)
     */
    private double getCorrectedY() {
        if (drive == null) return AUTON_START_Y;
        return drive.getY();
    }

    /**
     * Update corrected position - now just calls drive.updateOdometry()
     * Transformation is handled in SixWheelDriveController
     */
    private void updateCorrectedPosition() {
        // Transformation now handled in SixWheelDriveController.updateOdometry()
        // Nothing extra needed here
    }

    /**
     * Update all subsystem controllers
     */
    private void updateAllSystems() {
        // Update odometry and corrected position first
        if (drive != null) {
            drive.updateOdometry();
            updateCorrectedPosition();
        }

        if (turret != null) turret.update();

        // Update turret field controller for auto-aim
        // Continuously recalculate target angle based on current position
        if (turretField != null && autoAimEnabled && drive != null) {
            // Continuously update the target angle from odometry + persistent offset
            double targetAngle = calculateAngleToTarget() + turretOffset;
            turretField.setTargetFieldAngle(targetAngle);

            turretField.update(drive.getHeadingDegrees());
            turretError = turretField.getFieldError();
        }

        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }
}