package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.champion.RobotState;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;



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
 * A Button:       CLOSE preset - calculates turret angle to (0,0) based on position
 * X Button:       Set RPM to IDLE (2000 RPM)
 */
@Config
@TeleOp(name = "Undefeated Super Powerful Teleop Blue", group = "Competition")
public class MyOnlyTeleop1 extends LinearOpMode {

    // === PRESETS ===
    public static double FAR_RPM = 4100.0;
    public static double FAR_RAMP_ANGLE = 0.6;
    public static double FAR_TURRET = -23;
    // FAR_TURRET_ANGLE removed - now calculated dynamically

    public static double CLOSE_RPM = 3100.0;
    public static double CLOSE_RAMP_ANGLE = 0.449;
    public static double CLOSE_TURRET = -45;
    // CLOSE_TURRET_ANGLE removed - now calculated dynamically

    public static double IDLE_RPM = 2000.0;

    // === TARGET POSITION (where turret aims at) ===
    public static double TARGET_X = 0.0;  // Field X coordinate to aim at
    public static double TARGET_Y = 0.0;  // Field Y coordinate to aim at

    // === AUTON STARTING POSITION ===
    public static double AUTON_START_X = 49.6;
    public static double AUTON_START_Y = 9.0;
    public static double AUTON_START_HEADING = 0.0;  // degrees

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = 0.05;  // Always positive, direction handled in code
    public static double TURRET_TARGET_INCREMENT = 1.0;
    public static double RPM_INCREMENT = 100.0;
    public static double TURRET_MANUAL_SENSITIVITY = 0.2;

    // === BALL DETECTION ===
    // Switch reads 3.3V when not pressed, 0V when pressed (ball detected)
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // === TELEMETRY ===
    public static double TELEMETRY_INTERVAL_MS = 100;

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
    private final ElapsedTime telemetryTimer = new ElapsedTime();

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

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Read the static pose from auton
        Pose2d autonPose = PoseStorage.currentPose;

        telemetry.addLine("=== POSE FROM STORAGE ===");
        telemetry.addData("Auton Final Pose", "x=%.2f, y=%.2f, h=%.2f°",
                autonPose.position.x, autonPose.position.y,
                Math.toDegrees(autonPose.heading.toDouble()));
        telemetry.update();

        initializeHardware();

        sleep(300);

        // Set position from auton (transformation now handled in SixWheelDriveController)
        if (drive != null) {
            drive.setPosition(
                    autonPose.position.x,
                    autonPose.position.y,
                    autonPose.heading.toDouble()
            );
            drive.updateOdometry();

            telemetry.addData("Position SET", "x=%.1f, y=%.1f, h=%.1f°",
                    drive.getX(), drive.getY(), drive.getHeadingDegrees());
        }
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize turret angle tracking AFTER start
        if (turret != null) {
            turret.restoreAngle(PoseStorage.turretAngle);
            telemetry.addData("Turret RESTORED", "%.1f°", PoseStorage.turretAngle);
            telemetry.update();
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
        telemetryTimer.reset();

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

            // === SLOW UPDATES (telemetry) ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                updateTelemetry();
            }
        }

        // Cleanup
        if (shooter != null) shooter.stopShooting();
        if (turretField != null) turretField.disable();
    }

    /**
     * Initialize hardware
     */
    private void initializeHardware() {
        // Drive controller
        try {
            drive = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addLine("WARNING: Drive controller failed to initialize");
        }

        // Turret
        try {
            turret = new TurretController(this);
            turretField = new TurretFieldController(turret);
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Turret: " + e.getMessage());
        }

        // Ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Ramp: " + e.getMessage());
        }

        // Intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Intake: " + e.getMessage());
        }
        intake = new NewIntakeController(intakeMotor);

        // Transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Transfer: " + e.getMessage());
        }
        transfer = new NewTransferController(transferMotor);

        // Uptake
        CRServo uptakeServo = null;
        CRServo uptakeServo2 = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "servo1");
            uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake: " + e.getMessage());
        }
        uptake = new UptakeController(uptakeServo, uptakeServo2);

        // Uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake Switch: " + e.getMessage());
        }

        // Shooter
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Shooter: " + e.getMessage());
        }
        shooter = new NewShooterController(shooterMotor1,shooterMotor2);
    }

    /**
     * Sensitivity curve - CUBIC
     */
    private double applySensitivityCurve(double input) {
        return input * input * input;
    }

    /**
     * Drive controls - ARCADE DRIVE (FULL POWER)
     * Left Stick Y = Forward/Backward
     * Right Stick X = Turn
     */
    private void handleDriveControls() {
        if (drive == null) return;

        // Update odometry
        drive.updateOdometry();

        // Set to power mode for direct control
        drive.setDriveMode(SixWheelDriveController.DriveMode.POWER);

        // Arcade drive: left stick Y for drive, right stick X for turn (FULL POWER)
        double drivePower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;

        // Calculate left and right powers
        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;

        // Use tank drive at full power (no multipliers)
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

        // === DPAD RIGHT: TURRET TARGET +1° (only when auto-aim enabled) ===
        boolean currentDpadRight2 = gamepad2.dpad_right;
        if (currentDpadRight2 && !lastDpadRight2) {
            if (autoAimEnabled && turretField != null) {
                turretField.setTargetFieldAngle(turretField.getTargetFieldAngle() + TURRET_TARGET_INCREMENT);
            }
        }
        lastDpadRight2 = currentDpadRight2;

        // === DPAD LEFT: TURRET TARGET -1° (only when auto-aim enabled) ===
        boolean currentDpadLeft2 = gamepad2.dpad_left;
        if (currentDpadLeft2 && !lastDpadLeft2) {
            if (autoAimEnabled && turretField != null) {
                turretField.setTargetFieldAngle(turretField.getTargetFieldAngle() - TURRET_TARGET_INCREMENT);
            }
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
                turretField.setTargetFieldAngle(FAR_TURRET);
                turretField.enable();
                autoAimEnabled = true;
            }
        }
        lastY2 = currentY2;

        // === A: CLOSE PRESET (always activates) ===
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            // Activate CLOSE preset
            currentPresetMode = PresetMode.CLOSE;
            currentTargetRPM = CLOSE_RPM;
            if (shooter != null) {
                shooter.setTargetRPM(CLOSE_RPM);
                if (!shooter.isShootMode()) shooter.startShooting();
            }
            if (ramp != null) ramp.setTargetAngle(CLOSE_RAMP_ANGLE);
            if (turretField != null) {
                // Calculate angle to target based on current robot position
                double targetAngle = calculateAngleToTarget();
                turretField.setTargetFieldAngle(CLOSE_TURRET);
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
     * Calculate the field angle from robot position to target (0,0)
     * Returns the angle in degrees that the turret should face
     *
     * Uses atan2 to get angle from robot (x,y) to target (TARGET_X, TARGET_Y)
     * Result is in field coordinates (0° = forward on field)
     */
    private double calculateAngleToTarget() {
        double robotX = getCorrectedX();
        double robotY = getCorrectedY();

        // Vector from robot to target
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        // Calculate angle using atan2 (returns radians, -PI to PI)
        // atan2(dy, dx) gives angle where 0° = positive X axis
        // Convert to field coordinates where 0° = positive Y axis (forward)
        double angleRad = Math.atan2(dx, dy);  // Note: swapped dx,dy for field coords
        double angleDeg = Math.toDegrees(angleRad);

        return angleDeg;
    }

    /**
     * Get distance from robot to target
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
        if (turretField != null && autoAimEnabled && drive != null) {
            turretField.update(drive.getHeadingDegrees());
            turretError = turretField.getFieldError();
        }

        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }

    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        // Drive info - corrected field position
        telemetry.addData("Position", "X:%.1f Y:%.1f", getCorrectedX(), getCorrectedY());
        if (drive != null) {
            telemetry.addData("Heading", "%.1f°", drive.getHeadingDegrees());
        }

        // Shooter info
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
        }

        // Ramp info
        if (ramp != null) {
            telemetry.addData("Ramp", "%.3f → %.3f", ramp.getCurrentAngle(), ramp.getTargetAngle());
        }

        // Turret info
        if (turretField != null) {
            if (autoAimEnabled) {
                telemetry.addData("Turret Target", "%.1f° (err: %.1f°)",
                        turretField.getTargetFieldAngle(), turretField.getFieldError());

            }
        }


        telemetry.update();
    }
}