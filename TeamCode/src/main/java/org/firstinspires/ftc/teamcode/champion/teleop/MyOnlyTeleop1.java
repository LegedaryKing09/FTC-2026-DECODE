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
 * FIELD-CENTRIC AUTO-AIM:
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
    // Y button manual override (disables auto, locks to these values)
    public static double MANUAL_FAR_RPM = 4400.0;
    public static double MANUAL_FAR_RAMP = 0.34;

    public static double STARTING_RPM = 3500.0;

    // === SHOOTING POWER REDUCTION ===
    // When Driver 2 holds RT to shoot, intake and transfer power is reduced
    // by this fraction to free up battery headroom for the shooter wheels.
    public static double INTAKE_POWER_REDUCTION = 0.50;  // 30% reduction
    public static double TRANSFER_POWER_REDUCTION = 0.25;

    // === SHOOTING CURVE (parabolic interpolation) ===
    // 28in  → rpm=3250, ramp=0.0
    // 64in  → rpm=3450, ramp=0.16
    // 100in → rpm=3800, ramp=0.3
    // >100in → rpm=4200, ramp=0.35
    // Shooter target RPM = calculated RPM + RPM_READY (200)
    // Uptake gate threshold  = calculated RPM (exact)

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
    public static double RESET_HEADING = 0;  // degrees

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = 0.01;  // Always positive, direction handled in code
    public static double RPM_INCREMENT = 100.0;
    public static double TURRET_OFFSET_SPEED = 1;  // degrees per loop when dpad held

    // === CLOSE MODE AUTO-UPDATE ===
    // When in CLOSE preset, RPM and ramp auto-recalculate based on distance at this interval.
    // 500ms is a good balance: fast enough to track driving, slow enough to let PID settle.
    // Lower (250ms) = more responsive but PID may not fully settle between updates.
    // Higher (1000ms) = smoother PID but sluggish response when driving.
    public static double CLOSE_UPDATE_INTERVAL_MS = 250.0;

    // === RAMP FREEZE AFTER SHOT ===
    // After a ball clears the uptake switch, ramp updates are paused for this long.
    // Prevents the ramp from readjusting in response to the RPM dip during launch.
    public static double RAMP_FREEZE_DURATION_MS = 250.0;

    // === BALL DETECTION ===
    // Switch reads 3.3V when not pressed, 0V when pressed (ball detected)
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;
    public static double RPM_READY=200;

    // === CONTROLLERS ===
    private SixWheelDriveController drive;
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;
    private LimelightAlignmentController limelightController;
    private CRServo led;

    // RPM permanent offset (dpad up/down adjusts this, applied on top of auto RPM)
    private double rpmOffset = 0;

    // Speed threshold — only recalculate RPM/ramp when robot is nearly stopped
    public static double SPEED_THRESHOLD = 0.15;  // joystick magnitude below this = "stopped"

    // Ball detection switch
    private AnalogInput uptakeSwitch;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime closeUpdateTimer = new ElapsedTime();
    private final ElapsedTime rampFreezeTimer = new ElapsedTime();

    // Ramp freeze state (set when a ball passes through the uptake switch)
    private boolean rampFrozen = false;
    private boolean lastBallPresent = false;

    // === GAMEPAD 1 BUTTON STATES ===
    private boolean lastRB1 = false;
    private boolean lastX1 = false;
    private boolean lastB1 = false;

    // === GAMEPAD 2 BUTTON STATES ===
    private boolean lastY2 = false;
    private boolean lastA2 = false;
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

            // === BALL-PASSED DETECTION: freeze ramp briefly after each shot ===
            // Detects the falling edge on the uptake switch (ball leaves the switch).
            // Prevents the ramp from readjusting in response to the RPM dip during launch.
            if (uptakeSwitch != null) {
                boolean ballPresent = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
                if (lastBallPresent && !ballPresent) {
                    rampFrozen = true;
                    rampFreezeTimer.reset();
                }
                lastBallPresent = ballPresent;
            }
            if (rampFrozen && rampFreezeTimer.milliseconds() >= RAMP_FREEZE_DURATION_MS) {
                rampFrozen = false;
            }

            // === AUTO ZONE SWITCHING: Only recalculate when robot is nearly stopped ===
            // This lets the shooter PID settle before shooting and reduces battery drain.
            double driveSpeed = Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x);

            if (autoZoneSwitching
                    && driveSpeed < SPEED_THRESHOLD
                    && closeUpdateTimer.milliseconds() >= CLOSE_UPDATE_INTERVAL_MS) {
                double distance = getDistanceToTarget();
                double[] params = getShootingParamsForDistance(distance);
                double autoRPM = params[0] + RPM_READY + rpmOffset;
                double autoRamp = params[1];

                currentTargetRPM = autoRPM;
                if (shooter != null) shooter.setTargetRPM(autoRPM);
                if (ramp != null && !rampFrozen) ramp.setTargetAngle(autoRamp);

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

    /**
     * Returns the exact calculated RPM for the given distance.
     * Used to gate the uptake: balls only feed into the shooter once this is met.
     */
    private double getMinRPMForDistance(double distance) {
        return getShootingParamsForDistance(distance)[0];
    }

    /**
     * Get shooting parameters for a given distance using a parabolic curve.
     * Returns double[2]: [0] = calculated RPM (threshold), [1] = ramp angle.
     *
     * Fitted through three data points:
     *   28in  → RPM 3250, ramp 0.0
     *   64in  → RPM 3450, ramp 0.16
     *   100in → RPM 3800, ramp 0.3
     * >100in  → RPM 4200, ramp 0.35
     *
     * Callers must add RPM_READY (200) to [0] when setting the shooter target RPM.
     * The threshold for uptake gating is [0] exactly.
     */
    private double[] getShootingParamsForDistance(double distance) {
        double rpm, rampAngle;
        if (distance <= 28.0) {
            rpm = 3250.0;
            rampAngle = 0.0;
        } else if (distance <= 100.0) {
            double x = distance;
            // Quadratic fit: RPM through (28,3250), (64,3450), (100,3800)
            rpm = (25.0/432.0)*x*x + (25.0/108.0)*x + (86350.0/27.0);
            // Quadratic fit: ramp through (28,0.0), (64,0.16), (100,0.3)
            rampAngle = (-1.0/129600.0)*x*x + (167.0/32400.0)*x + (-56.0/405.0);
        } else {
            rpm = 4200.0;
            rampAngle = 0.35;
        }
        return new double[]{rpm, rampAngle};
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

        // Turret
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

        // LED indicator (Spark Mini on servo port, no motor attached)
        try {
            led = hardwareMap.get(CRServo.class, "led");
        } catch (Exception e) {
            // LED not configured - led stays null
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
        currentPresetMode = PresetMode.CLOSE;
        rpmOffset = 0;
    }

    /**
     * Driver 2 controls
     */
    @SuppressLint("DefaultLocale")
    private void handleDriver2Controls() {

        // === RT: HOLD TO SHOOT - Run intake/transfer; gate uptake on shooter RPM ===
        // Intake and transfer start immediately so balls are staged.
        // Uptake only runs once shooter RPM >= zone minimum (RPM_A), preventing
        // balls from entering the shooter before it's up to speed.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // Start intake and transfer on first press
            if (!allSystemsFromTrigger) {
                if (intake != null) {
                    intake.power = intake.power * (1.0 - INTAKE_POWER_REDUCTION);
                }
                if (transfer != null) {
                    transfer.power = transfer.power * (1.0 - TRANSFER_POWER_REDUCTION);
                }
                if (intake != null && !intake.isActive()) {
                    intake.reversed = false;
                    intake.toggle();
                }
                if (transfer != null && !transfer.isActive()) {
                    transfer.reversed = false;
                    transfer.toggle();
                }
                allSystemsFromTrigger = true;
            }

            // Every loop: gate uptake on shooter RPM being within RPM_READY of target
            // This is more flexible for different battery situations — shoots when close enough
            boolean rpmReady = shooter != null && shooter.getRPM() >= (currentTargetRPM - RPM_READY);
            if (rpmReady) {
                if (uptake != null && !uptake.isActive()) {
                    uptake.reversed = false;
                    uptake.toggle();
                }
            } else {
                if (uptake != null && uptake.isActive()) {
                    uptake.toggle();
                }
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

        if (lastLLTelemetry != null) {
            telemetry.addLine(lastLLTelemetry);
        }


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

        // === DPAD UP: RPM OFFSET +100 (permanent, applied on top of auto RPM) ===
        boolean currentDpadUp2 = gamepad2.dpad_up;
        if (currentDpadUp2 && !lastDpadUp2) {
            rpmOffset += RPM_INCREMENT;
            currentTargetRPM += RPM_INCREMENT;
            if (shooter != null) shooter.setTargetRPM(currentTargetRPM);
        }
        lastDpadUp2 = currentDpadUp2;

        // === DPAD DOWN: RPM OFFSET -100 ===
        boolean currentDpadDown2 = gamepad2.dpad_down;
        if (currentDpadDown2 && !lastDpadDown2) {
            rpmOffset -= RPM_INCREMENT;
            currentTargetRPM -= RPM_INCREMENT;
            if (shooter != null) shooter.setTargetRPM(currentTargetRPM);
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


        // === RIGHT STICK X: MANUAL TURRET NUDGE (works with or without auto-aim) ===
        // When auto-aim is on: adds to the aim offset so manual tweaks layer on top
        // When auto-aim is off: directly steps the turret servo position
        double turretStick = gamepad2.right_stick_x;
        if (Math.abs(turretStick) > 0.1 && turret != null) {
            if (turret.isAutoAimEnabled()) {
                // Auto-aim on: nudge the aim offset (same as dpad but analog)
                turret.addAimOffset(-turretStick * TURRET_OFFSET_SPEED);
            } else {
                // Auto-aim off: directly move the servo position
                double currentPos = turret.getCommandedPosition();
                double newPos = currentPos + (turretStick * 0.005);  // Scale for fine control
                newPos = Math.max(0.0, Math.min(1.0, newPos));
                turret.setServoPosition(newPos);
            }
        }

        // === Y: MANUAL FAR OVERRIDE (locks RPM/ramp, disables auto zone switching) ===
        // Use when auto calculation is wrong and you need to force a far shot
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            currentPresetMode = PresetMode.FAR;
            autoZoneSwitching = false;  // DISABLE auto — lock to manual values
            currentTargetRPM = MANUAL_FAR_RPM + rpmOffset;
            if (shooter != null) {
                shooter.setTargetRPM(currentTargetRPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(MANUAL_FAR_RAMP);
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
            double autoRPM = params[0] + RPM_READY + rpmOffset;
            double autoRamp = params[1];

            // Apply the calculated RPM and ramp
            currentTargetRPM = autoRPM;
            if (shooter != null) {
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
    }
    public static double DISTANCE_X = 10.0;  // Field X coordinate to aim at
    public static double DISTANCE_Y = 10.0;  // Field Y coordinate to aim at
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

        // Phase: 0=ON, 1=ON, 2=OFF, 3=OFF  → slow flash (500ms on, 500ms off) = phases 0,1
        // Phase: 0=ON, 1=OFF, 2=ON, 3=OFF  → fast flash (250ms on/off)        = phases 0,2
        // Solid: all phases on
        // Off: all phases off
        if (led != null) {
            int phase = (int) ((runtime.milliseconds() / 250) % 4);

            boolean ballLoaded = uptakeSwitch != null
                    && uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

            if (!ballLoaded) {
                // No ball — off
                led.setPower(0.0);
            } else {
                double rpmError = shooter != null
                        ? shooter.getRPM() - shooter.getTargetRPM() : 0;

                if (rpmError > 300) {
                    // Too high — fast flash green — phases 0,2
                    boolean on = (phase == 0 || phase == 2);
                    led.setPower(on ? 0.8 : 0.0);
                } else if (rpmError < -100) {
                    // Too low — slow flash green — phases 0,1
                    boolean on = (phase == 0 || phase == 1);
                    led.setPower(on ? 0.8 : 0.0);
                } else {
                    // Ready — solid green
                    led.setPower(0.8);
                }
            }
        }
    }
}