package org.firstinspires.ftc.teamcode.champion.teleop;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;

/**
 * 超级无敌高级Teleop
 * FIELD-CENTRIC AUTO-AIM (NO TurretFieldController):
 * - Inherits position from auton via PoseStorage
 * - Continuously tracks robot position on field
 * - Auto-calculates FIELD angle to target, subtracts robot heading to get
 *   turret-relative angle, then converts directly to servo position:
 *     servoPos = 0.5 + (turretAngle / 315.0)
 * - Runs every loop for continuous aiming while driving
 * TURRET SERVO MAPPING:
 * - Servo 0.0 to 1.0 = 315° total range
 * - Servo 0.5 = 0° (field forward)
 * - Servo 0.0 = -157.5°, Servo 1.0 = +157.5°
 * === DRIVER 1 (gamepad1) - ARCADE DRIVE (FULL POWER) ===
 * Left Stick Y:   Forward/Backward (1.0 power)
 * Right Stick X:  Turn Left/Right (1.0 power)
 * Right Bumper:   Toggle intake + transfer + uptake (all three)
 *                 Ball switch auto-stops uptake when ball detected,
 *                 auto-restarts when ball leaves
 * Left Bumper:    HOLD to VOMIT (reverse intake + transfer + uptake)
 * X Button:       RESET ODOMETRY to known position (for auton crash recovery)
 * B Button:       EMERGENCY RESET all subsystems (stops everything, resets states)
 * === DRIVER 2 (gamepad2) ===
 * Right Trigger:  HOLD to SHOOT - runs intake + transfer + uptake,
 *                 BYPASSES ball detection (forces balls through shooter)
 * Left Bumper:    RETRACT ramp (angle goes more negative)
 * Left Trigger:   EXTEND ramp (angle goes less negative, toward 0°)
 * D-Pad Up:       RPM +100
 * D-Pad Down:     RPM -100
 * D-Pad Right:    HOLD to nudge turret aim offset right (continuous)
 * D-Pad Left:     HOLD to nudge turret aim offset left (continuous)
 * Right Stick X:  Manual turret - DISABLES AUTO-AIM when moved!
 *                 Auto-aim stays off until next preset button is pressed
 * Y Button:       FAR preset - enables auto-aim + auto zone switching
 * A Button:       CLOSE preset - enables auto-aim + auto zone switching
 *                 (auto switches between CLOSE/FAR based on distance)
 * X Button:       Toggle RPM BOOST (+150 RPM for low battery compensation)
 */
@Config
@TeleOp(name = "Undefeated Super Powerful Teleop Blue", group = "Competition")
public class MyOnlyTeleop1 extends LinearOpMode {

    // === PRESETS ===
    public static double FAR_RPM = 4500.0;
    public static double FAR_RAMP_ANGLE = 0.18;

    public static double STARTING_RPM = 3900.0;

    // === DISTANCE-BASED AUTO ZONE SWITCHING ===
    // CLOSE zone: 0 to CLOSE_MAX_DISTANCE inches (uses distance table for RPM/ramp)
    // FAR zone: FAR_MIN_DISTANCE to FAR_MAX_DISTANCE inches (uses FAR_RPM/FAR_RAMP_ANGLE)
    // Dead zone: between CLOSE_MAX and FAR_MIN — holds last mode (no switching)
    public static double CLOSE_MAX_DISTANCE = 90.0;
    public static double FAR_MIN_DISTANCE = 100.0;
    public static double FAR_MAX_DISTANCE = 160.0;

    // === CLOSE MODE RPM BOOST ===
    // X button toggles this boost ON/OFF. When active, all close-preset
    // RPM values are increased by this amount to compensate for battery voltage drop.
    public static double CLOSE_RPM_BOOST = 100.0;

    // === SHOOTING POWER REDUCTION ===
    // When Driver 2 holds RT to shoot, intake and transfer power is reduced
    // by this fraction to free up battery headroom for the shooter wheels.
    public static double SHOOTING_POWER_REDUCTION = 0.50;  // 30% reduction

    // === DISTANCE-BASED SHOOTING TABLE ===
    // Distance (inches) = straight-line odometry distance to target
    // Each entry: {distance, ramp angle, RPM}
    // Piecewise: if distance is within ±6 inches of a calibrated point, use that setting
    public static double DISTANCE_TOLERANCE = 6.0;  // ±6 inches

    // Calibration points (straight-line odometry distance to target)
    public static double DIST_1 = 24.0;   public static double RAMP_1 = 0;  public static double RPM_1 = 3600;
    public static double DIST_2 = 36.0;   public static double RAMP_2 = 0.2;  public static double RPM_2 = 3650;
    public static double DIST_3 = 48.0;   public static double RAMP_3 = 0.28;  public static double RPM_3 = 3800;
    public static double DIST_4 = 60.0;   public static double RAMP_4 = 0.4;  public static double RPM_4 = 3900;
    public static double DIST_5 = 72.0;   public static double RAMP_5 = 0.37;  public static double RPM_5 = 3950;

    // === TARGET POSITION (where turret aims at) ===
    public static double TARGET_X = 10.0;  // Field X coordinate to aim at
    public static double TARGET_Y = 10.0;  // Field Y coordinate to aim at

    // === AUTON STARTING POSITION ===
    public static double AUTON_START_X = 48;
    public static double AUTON_START_Y = 137;
    public static double AUTON_START_HEADING = 0.0;  // degrees

    // === MANUAL ODOMETRY RESET POSITION (Driver 1 X button) ===
    // Set this to the known position where you place the robot if auton crashes.
    // Defaults to auton start position. Tune via Dashboard if different.
    public static double RESET_X = 48;
    public static double RESET_Y = 137;
    public static double RESET_HEADING = 0.0;  // degrees

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = 0.02;  // Always positive, direction handled in code
    public static double RPM_INCREMENT = 100.0;
    public static double TURRET_OFFSET_SPEED = 1;  // degrees per loop when dpad held

    // === CLOSE MODE AUTO-UPDATE ===
    // When in CLOSE preset, RPM and ramp auto-recalculate based on distance at this interval.
    // 500ms is a good balance: fast enough to track driving, slow enough to let PID settle.
    // Lower (250ms) = more responsive but PID may not fully settle between updates.
    // Higher (1000ms) = smoother PID but sluggish response when driving.
    public static double CLOSE_UPDATE_INTERVAL_MS = 200.0;

    // === BALL DETECTION ===
    // Switch reads 3.3V when not pressed, 0V when pressed (ball detected)
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // === CONTROLLERS ===
    private SixWheelDriveController drive;
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;
    private LimelightAlignmentController limelightController;

    // Ball detection switch
    private AnalogInput uptakeSwitch;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime closeUpdateTimer = new ElapsedTime();

    // === GAMEPAD 1 BUTTON STATES ===
    private boolean lastRB1 = false;
    private boolean lastX1 = false;
    private boolean lastB1 = false;

    // === GAMEPAD 2 BUTTON STATES ===
    private boolean lastY2 = false;
    private boolean lastA2 = false;
    private boolean lastX2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastLB2 = false;
    private boolean lastLT2Pressed = false;

    private boolean lastGamepad2B = false;
    private String lastLLTelemetry = null;

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.2;

    // === STATE TRACKING ===
    private boolean intakeModeActive = false;
    private boolean allSystemsFromTrigger = false;
    private double currentTargetRPM = 0;

    // === Preset mode tracking ===
    private enum PresetMode { FAR, CLOSE }
    private PresetMode currentPresetMode = PresetMode.CLOSE;
    private boolean closeRpmBoostActive = false;
    private boolean autoZoneSwitching = false;  // Enabled when A or Y is pressed

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

        // Initialize turret to center (0° field angle)
        if (turret != null) {
            turret.setServoPosition(0.5);
            turret.setTarget(TARGET_X, TARGET_Y);
        }

        // Initialize ramp
        if (ramp != null) {
            ramp.initialize();
        }

        // Auto-aim starts DISABLED
        if (turret != null) {
            turret.disableAutoAim();
        }

        // Start shooter at starting RPM
        if (shooter != null) {
            shooter.setTargetRPM(STARTING_RPM);
            currentTargetRPM = STARTING_RPM;
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

            // === AUTO ZONE SWITCHING: Automatically switch between CLOSE and FAR ===
            // based on distance to target. Runs at CLOSE_UPDATE_INTERVAL_MS.
            if (autoZoneSwitching
                    && closeUpdateTimer.milliseconds() >= CLOSE_UPDATE_INTERVAL_MS) {
                double distance = getDistanceToTarget();

                if (distance <= CLOSE_MAX_DISTANCE) {
                    // CLOSE zone — use distance table
                    if (currentPresetMode != PresetMode.CLOSE) {
                        currentPresetMode = PresetMode.CLOSE;
                        if (shooter != null) {
                            shooter.setShootMode(NewShooterController.ShootMode.CLOSE);
                        }
                    }
                    double[] params = getShootingParamsForDistance(distance);
                    double autoRPM = params[0] + (closeRpmBoostActive ? CLOSE_RPM_BOOST : 0);
                    double autoRamp = params[1];

                    currentTargetRPM = autoRPM;
                    if (shooter != null) shooter.setTargetRPM(autoRPM);
                    if (ramp != null) ramp.setTargetAngle(autoRamp);

                } else if (distance >= FAR_MIN_DISTANCE && distance <= FAR_MAX_DISTANCE) {
                    // FAR zone — use fixed FAR preset
                    if (currentPresetMode != PresetMode.FAR) {
                        currentPresetMode = PresetMode.FAR;
                        currentTargetRPM = FAR_RPM;
                        if (shooter != null) {
                            shooter.setShootMode(NewShooterController.ShootMode.FAR);
                            shooter.setTargetRPM(FAR_RPM);
                        }
                        if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);
                    }
                }
                // Dead zone (between CLOSE_MAX and FAR_MIN): hold current mode, no changes

                closeUpdateTimer.reset();
            }

            // Update all controllers
            updateAllSystems();

            // === TELEMETRY ===
            updateTelemetry();
        }

        // Cleanup
        if (shooter != null) shooter.stopShooting();
    }

    // =========================================================================
    // DISTANCE / SHOOTING PARAMS
    // =========================================================================

    /**
     * Get RPM and Ramp angle based on distance using piecewise function
     * Returns double[2] where [0] = RPM, [1] = ramp angle
     * Distance tolerance is ±6 inches from calibrated points
     */
    private double[] getShootingParamsForDistance(double distance) {
        double[] distances = {DIST_1, DIST_2, DIST_3, DIST_4, DIST_5};
        double[] rpm = {RPM_1, RPM_2, RPM_3, RPM_4, RPM_5};
        double[] ramps = {RAMP_1, RAMP_2, RAMP_3, RAMP_4, RAMP_5};

        // Check each calibration point with tolerance
        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distance - distances[i]) <= DISTANCE_TOLERANCE) {
                return new double[]{rpm[i], ramps[i]};
            }
        }

        // No match - use closest point
        double minDiff = Double.MAX_VALUE;
        int bestIdx = 2;
        for (int i = 0; i < distances.length; i++) {
            double diff = Math.abs(distance - distances[i]);
            if (diff < minDiff) {
                minDiff = diff;
                bestIdx = i;
            }
        }
        return new double[]{rpm[bestIdx], ramps[bestIdx]};
    }

    /**
     * Update telemetry with ramp, RPM, turret angle, and odometry
     */
    private void updateTelemetry() {
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
        }
        if (ramp != null) {
            telemetry.addData("Ramp Angle", "%.3f", ramp.getTargetAngle());
        }
        telemetry.addData("Mode", "%s", currentPresetMode);
        if (turret != null) {
            telemetry.addData("Turret", turret.getCommandedPosition());
        }
        telemetry.addData("Robot X", "%.1f", getCorrectedX());
        telemetry.addData("Robot Y", "%.1f", getCorrectedY());
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

        // Turret (no TurretFieldController needed)
        try {
            turret = new TurretController(this);
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
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);

        // Limelight
        try {
            limelightController = new LimelightAlignmentController(this, drive);
        } catch (Exception e) {
            // Limelight init failed - limelightController stays null
        }
    }

    // === DRIVE SENSITIVITY ===
    public static double DRIVE_EXPONENT = 2.0;  // 1.0 = linear, 2.0 = squared, 3.0 = cubed
    public static double TURN_EXPONENT = 4.0;   // Higher = more precision at low speeds
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
            intakeModeActive = false;

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

        // === X: RESET ODOMETRY to known position ===
        // Use when auton crashed and you need to manually set robot position
        boolean currentX1 = gamepad1.x;
        if (currentX1 && !lastX1) {
            if (drive != null) {
                drive.setPosition(RESET_X, RESET_Y, Math.toRadians(RESET_HEADING));
                drive.updateOdometry();
            }
            // Reset turret aim offset since position changed
            if (turret != null) {
                turret.setAimOffset(0);
            }
        }
        lastX1 = currentX1;

        // === B: EMERGENCY RESET ALL SUBSYSTEMS (except odometry) ===
        // Stops all motors, resets all states, restarts shooter at starting RPM.
        // Use when robot is acting erratic or subsystems are in a bad state.
        boolean currentB1 = gamepad1.b;
        if (currentB1 && !lastB1) {
            emergencyReset();
        }
        lastB1 = currentB1;
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
     * PRIORITY: If driver 2 is holding RT (shooting), do NOT stop ANY systems.
     * Driver 2 shooting has full priority - intake, transfer, uptake all stay running.
     * They will be cleaned up when driver 2 releases RT instead.
     */
    private void stopIntakeMode() {
        intakeModeActive = false;

        // If driver 2 is shooting, don't touch anything - let RT control everything
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        if (shooting) return;

        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    /**
     * EMERGENCY RESET - stops all subsystems and resets all states.
     * Does NOT reset odometry (use Driver 1 X for that).
     * Brings the robot back to a clean startup state so you can recover mid-match.
     */
    private void emergencyReset() {
        // Stop all feed systems
        if (intake != null) {
            intake.reversed = false;
            intake.setState(false);
            intake.power = -1.0;  // Restore full power
        }
        if (transfer != null) {
            transfer.reversed = false;
            transfer.setState(false);
            NewTransferController.power = -1.0;  // Restore full power
        }
        if (uptake != null) {
            uptake.reversed = false;
            uptake.setState(false);
        }

        // Reset shooter to starting RPM
        if (shooter != null) {
            shooter.setTargetRPM(STARTING_RPM);
            shooter.setShootMode(NewShooterController.ShootMode.CLOSE);
            if (!shooter.isShootMode()) shooter.startShooting();
        }
        currentTargetRPM = STARTING_RPM;

        // Reset ramp
        if (ramp != null) {
            ramp.initialize();
        }

        // Reset turret to center, clear offset, disable auto-aim
        if (turret != null) {
            turret.setServoPosition(0.5);
            turret.setAimOffset(0);
            turret.disableAutoAim();
        }

        // Reset all state flags
        intakeModeActive = false;
        allSystemsFromTrigger = false;
        autoZoneSwitching = false;
        closeRpmBoostActive = false;
        currentPresetMode = PresetMode.CLOSE;

        // Rumble gamepad to confirm reset
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }

    /**
     * Driver 2 controls
     */
    @SuppressLint("DefaultLocale")
    private void handleDriver2Controls() {

        // === RT: HOLD TO SHOOT - Run ITU and bypass ball detection ===
        // This forces balls through the shooter regardless of the uptake switch
        // Also reduces intake/transfer power to give shooter more battery headroom
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // Start all three if not already running from trigger
            if (!allSystemsFromTrigger) {
                // Reduce intake/transfer power while shooting
                if (intake != null) {
                    intake.power = intake.power * (1.0 - SHOOTING_POWER_REDUCTION);
                }
                if (transfer != null) {
                    NewTransferController.power = NewTransferController.power * (1.0 - SHOOTING_POWER_REDUCTION);
                }

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
            // RT released - stop systems that were started by trigger
            if (allSystemsFromTrigger) {
                // Restore full intake/transfer power
                if (intake != null) {
                    intake.power = -1.0;
                }
                NewTransferController.power = -1.0;

                // Only stop intake/transfer if driver 1 intake mode is off
                if (!intakeModeActive) {
                    if (intake != null && intake.isActive()) intake.toggle();
                    if (transfer != null && transfer.isActive()) transfer.toggle();
                }
                if (uptake != null && uptake.isActive()) uptake.toggle();
                allSystemsFromTrigger = false;
            }
        }

        // Sticky limelight telemetry (persists until next B press)
        if (lastLLTelemetry != null) {
            telemetry.addLine(lastLLTelemetry);
        }

        // When driver 2 presses B: Limelight recalibration
        // Limelight is mounted at ROBOT CENTER facing forward (= turret servo 0.5).
        // tx = degrees the target is offset from robot center's forward direction.
        // So the correct turret servo position = 0.5 + (tx / 315).
        // Auto-aim currently has the turret at currentServo (which includes aimOffset).
        //
        // We want to find what aimOffset should be so that auto-aim puts the turret
        // at the correct position. Since auto-aim computes:
        //   finalServo = autoAimServo + (aimOffset / 315)
        // And we know:
        //   currentServo = autoAimServo + (oldAimOffset / 315)
        // Therefore:
        //   autoAimServo = currentServo - (oldAimOffset / 315)
        // We want:
        //   correctServo = autoAimServo + (newAimOffset / 315)
        // So:
        //   newAimOffset = (correctServo - autoAimServo) * 315
        //               = (correctServo - currentServo + oldAimOffset/315) * 315
        //               = (correctServo - currentServo) * 315 + oldAimOffset
        if (gamepad2.b && !lastGamepad2B) {
            if (limelightController != null && turret != null && limelightController.canSeeTarget()) {
                double tx = limelightController.getTx();
                double currentServo = turret.getCommandedPosition();
                double oldOffset = turret.getAimOffset();

                // Where the turret SHOULD be based on limelight
                double correctServo = 0.5 + (tx / TurretController.SERVO_RANGE_DEG);
                correctServo = Math.max(0.0, Math.min(1.0, correctServo));

                // Compute the new absolute offset
                double servoError = correctServo - currentServo;
                double newOffset = oldOffset + (servoError * TurretController.SERVO_RANGE_DEG);

                turret.setAimOffset(newOffset);

                lastLLTelemetry = String.format(
                        "LL tx=%.1f err=%.3f offset=%.1f",
                        tx, servoError, turret.getAimOffset());
            } else {
                lastLLTelemetry = "LL: No target visible";
            }
        }
        lastGamepad2B = gamepad2.b;

        // === LB: RETRACT RAMP (more negative angle, toward -245°) ===

        boolean ltPressed = gamepad2.left_bumper;
        if (ltPressed && !lastLT2Pressed) {
            if (ramp != null) {
                double newTarget = ramp.getTargetAngle() + RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
            }
        }

        lastLT2Pressed = ltPressed;

        // === LT: EXTEND RAMP (less negative angle, toward 0°) ===
        boolean currentLB2 = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (currentLB2 && !lastLB2) {
            if (ramp != null) {
                double newTarget = ramp.getTargetAngle() - RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
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
            if (currentTargetRPM < 0) currentTargetRPM = 0;
            if (shooter != null) {
                shooter.setTargetRPM(currentTargetRPM);
            }
        }
        lastDpadDown2 = currentDpadDown2;

        // === DPAD RIGHT: TURRET AIM OFFSET RIGHT (continuous while held) ===
        // Only if dpad_up/down not pressed (avoids accidental nudge when adjusting RPM)
        if (gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            if (turret != null) {
                turret.addAimOffset(-TURRET_OFFSET_SPEED);
            }
        }

        // === DPAD LEFT: TURRET AIM OFFSET LEFT (continuous while held) ===
        if (gamepad2.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_down) {
            if (turret != null) {
                turret.addAimOffset(TURRET_OFFSET_SPEED);
            }
        }


        // === Y: FAR PRESET (manual override + enables auto zone switching) ===
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            // Activate FAR preset
            currentPresetMode = PresetMode.FAR;
            currentTargetRPM = FAR_RPM;
            if (shooter != null) {
                shooter.setShootMode(NewShooterController.ShootMode.FAR);
                shooter.setTargetRPM(FAR_RPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);

            // Enable auto zone switching and auto-aim
            autoZoneSwitching = true;
            closeUpdateTimer.reset();
            if (turret != null) turret.enableAutoAim();
        }
        lastY2 = currentY2;

        // === A: CLOSE PRESET (manual override + enables auto zone switching) ===
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            currentPresetMode = PresetMode.CLOSE;

            // Calculate distance to goal and get shooting parameters
            double distance = getDistanceToTarget();
            double[] params = getShootingParamsForDistance(distance);
            double autoRPM = params[0] + (closeRpmBoostActive ? CLOSE_RPM_BOOST : 0);
            double autoRamp = params[1];

            // Apply the calculated RPM and ramp
            currentTargetRPM = autoRPM;
            if (shooter != null) {
                shooter.setShootMode(NewShooterController.ShootMode.CLOSE);
                shooter.setTargetRPM(autoRPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(autoRamp);

            // Enable auto zone switching and auto-aim
            autoZoneSwitching = true;
            closeUpdateTimer.reset();
            if (turret != null) turret.enableAutoAim();
        }
        lastA2 = currentA2;

        // === X: TOGGLE CLOSE RPM BOOST (+150 RPM for low battery) ===
        boolean currentX2 = gamepad2.x;
        if (currentX2 && !lastX2) {
            closeRpmBoostActive = !closeRpmBoostActive;

            // If currently in close mode, immediately apply/remove boost
            if (currentPresetMode == PresetMode.CLOSE) {
                double distance = getDistanceToTarget();
                double[] params = getShootingParamsForDistance(distance);
                double autoRPM = params[0] + (closeRpmBoostActive ? CLOSE_RPM_BOOST : 0);
                currentTargetRPM = autoRPM;
                if (shooter != null) shooter.setTargetRPM(autoRPM);
            }
        }
        lastX2 = currentX2;
    }
    public static double DISTANCE_X = 16.0;  // Field X coordinate to aim at
    public static double DISTANCE_Y = 18.0;  // Field Y coordinate to aim at
    /**
     * Get distance from robot to target (in inches)
     */
    private double getDistanceToTarget() {
        double robotX = getCorrectedX();
        double robotY = getCorrectedY();

        double dx = DISTANCE_X - robotX;
        double dy = DISTANCE_Y - robotY;

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
     * Update all subsystem controllers
     */
    private void updateAllSystems() {
        // Update odometry first
        if (drive != null) {
            drive.updateOdometry();
        }

        // Turret: either auto-aim or manual stepping, never both
        if (turret != null) {
            if (turret.isAutoAimEnabled() && drive != null) {
                turret.updateAutoAim(drive.getX(), drive.getY(), drive.getHeadingDegrees());
            } else {
                turret.update();
            }
        }

        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }
}