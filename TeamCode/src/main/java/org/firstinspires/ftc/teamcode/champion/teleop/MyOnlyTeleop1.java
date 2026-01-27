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



/**
 * 超级无敌高级Teleop
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
 * Right Trigger:  Hold to run intake + transfer + uptake (all three)
 *
 * Left Bumper:    EXTEND ramp (angle goes more negative)
 * Left Trigger:   RETRACT ramp (angle goes less negative)
 *
 * D-Pad Up:       RPM +100
 * D-Pad Down:     RPM -100
 * D-Pad Right:    Turret target +1 degree (when auto-aim enabled)
 * D-Pad Left:     Turret target -1 degree (when auto-aim enabled)
 *
 * Right Stick X:  Manual turret - DISABLES AUTO-AIM when moved!
 *                 Auto-aim stays off until next preset button is pressed
 *
 * Y Button:       FAR preset (4550 RPM, ramp, turret) + auto-aim ON
 * A Button:       CLOSE preset (3650 RPM, ramp, turret) + auto-aim ON
 * X Button:       Set RPM to IDLE (2000 RPM)
 */
@Config
@TeleOp(name = "超级无敌高级Teleop Blue", group = "Competition")
public class MyOnlyTeleop1 extends LinearOpMode {

    // === PRESETS ===
    public static double FAR_RPM = 4600.0;
    public static double FAR_RAMP_ANGLE = -175.0;
    public static double FAR_TURRET_ANGLE = 25.0;

    public static double CLOSE_RPM = 3900.0;
    public static double CLOSE_RAMP_ANGLE = -118.4;
    public static double CLOSE_TURRET_ANGLE = 45.0;

    public static double IDLE_RPM = 2000.0;

    // === ADJUSTMENTS ===
    public static double RAMP_INCREMENT_DEGREES = -10.0;
    public static double TURRET_TARGET_INCREMENT = 1.0;
    public static double RPM_INCREMENT = 100.0;
    public static double TURRET_MANUAL_SENSITIVITY = 0.5;

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

    @Override
    public void runOpMode() {
        Pose2d startPose;
        if (RobotState.isPoseValid()) {
            startPose = RobotState.getLastAutonPose();
            telemetry.addData("Starting from Auton", "x=%.1f, y=%.1f, heading=%.1f°",
                    startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.log()));
        } else {
            startPose = new Pose2d(0, 0, 0);
            telemetry.addData("Starting from", "Default (0,0,0)");
        }
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeHardware();
        // SET THE POSITION AFTER INITIALIZING HARDWARE
        if (drive != null) {
            drive.setPosition(
                    startPose.position.x,
                    startPose.position.y,
                    startPose.heading.log()
            );
            telemetry.addData("Drive Position Set", "Success");
        }
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize turret angle tracking AFTER start
        if (turret != null) {
            turret.initialize();
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
            if (intakeModeActive && uptakeSwitch != null) {
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
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);
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

        // === RT: HOLD TO RUN INTAKE + TRANSFER + UPTAKE (all three) ===
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

        // === LB: EXTEND RAMP (more negative angle) ===
        boolean currentLB2 = gamepad2.left_bumper;
        if (currentLB2 && !lastLB2) {
            if (ramp != null) {
                double currentAngle = ramp.getCurrentAngle();
                double newTarget = currentAngle + RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
                rampDebug = "LB: " + currentAngle + " -> " + newTarget;
            }
        }
        lastLB2 = currentLB2;

        // === LT: RETRACT RAMP (less negative angle) ===
        boolean ltPressed = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (ltPressed && !lastLT2Pressed) {
            if (ramp != null) {
                double currentAngle = ramp.getCurrentAngle();
                double newTarget = currentAngle - RAMP_INCREMENT_DEGREES;
                ramp.setTargetAngle(newTarget);
                rampDebug = "LT: " + currentAngle + " -> " + newTarget;
            }
        }
        lastLT2Pressed = ltPressed;

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
                turretField.setTargetFieldAngle(FAR_TURRET_ANGLE);
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
                turretField.setTargetFieldAngle(CLOSE_TURRET_ANGLE);
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
     * Update all subsystem controllers
     */
    private void updateAllSystems() {
        if (turret != null) turret.update();

        // Update turret field controller for auto-aim
        if (turretField != null && autoAimEnabled && drive != null) {
            turretField.update(drive.getHeadingDegrees());
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
        // Drive info
        if (drive != null) {
            telemetry.addData("Position", "X:%.1f Y:%.1f", drive.getX(), drive.getY());
            telemetry.addData("Heading", "%.1f°", drive.getHeadingDegrees());
        }

        // Shooter info
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
        }

        // Ramp info
        if (ramp != null) {
            telemetry.addData("Ramp", "%.1f° → %.1f°", ramp.getCurrentAngle(), ramp.getTargetAngle());
        }
        if (turretField != null) {
            if (autoAimEnabled) {
                telemetry.addData("Turret Target", "%.1f° (err: %.1f°)",
                        turretField.getTargetFieldAngle(), turretField.getFieldError());
            }
        }
        telemetry.update();
    }
}