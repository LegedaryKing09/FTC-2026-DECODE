package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@TeleOp(name = "December Teleop", group = "Competition")
public class DecemberTeleop extends LinearOpMode {

    // Ramp angle increment (tunable via FTC Dashboard)
    public static double RAMP_INCREMENT_DEGREES = 10.0;

    // Target AprilTag ID for turret alignment
    public static int TURRET_TARGET_TAG_ID = 20;

    // Shooter presets (tunable via FTC Dashboard)
    public static double CLOSE_RPM = 3650.0;
    public static double FAR_RPM = 4600.0;

    // RPM manual adjustment increment (tunable via FTC Dashboard)
    public static double RPM_INCREMENT = 50.0;

    // Idle RPM for shooter when intake system is stopped
    public static double IDLE_RPM = 2000.0;

    // Ramp angle presets (adjust these based on testing)
    public static double CLOSE_RAMP_ANGLE = 171.0;
    public static double FAR_RAMP_ANGLE = 92.0;

    // Telemetry update interval (ms) - reduces loop overhead
    public static double TELEMETRY_INTERVAL_MS = 100;  // 10Hz telemetry

    // Controllers
    private SixWheelDriveController drive;
    private TurretController turret;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // Timers
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // Loop timing stats
    private double loopTimeMs = 0;
    private double avgLoopTimeMs = 0;

    // David's gamepad (gamepad1) button states
    private boolean lastRightBumper1 = false;
    private boolean lastX1 = false;
    private boolean lastY1 = false;
    private boolean lastA1 = false;

    // Edward's gamepad (gamepad2) button states
    private boolean lastX2 = false;
    private boolean lastA2 = false;
    private boolean lastY2 = false;
    private boolean lastB2 = false;
    private boolean lastRightBumper2 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;

    // Trigger threshold (30%)
    private static final double TRIGGER_THRESHOLD = 0.3;

    // Intake mode state (all wheels together)
    private boolean intakeModeActive = false;

    // Vomit mode state (reverse all wheels)
    private boolean vomitModeActive = false;

    // Track if uptake was stopped by ball detection switch
    private boolean uptakeStoppedBySwitch = false;

    // Track current shooter target RPM for display
    private double currentTargetRPM = 0;

    // Track if uptake is running from Edward's right trigger
    private boolean uptakeFromTrigger = false;

    private boolean isShooting = false;

    @Override
    public void runOpMode() {
        // FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeHardware();

        telemetry.addLine("=== DECEMBER TELEOP ===");
        telemetry.addLine("Hardware initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize turret angle tracking AFTER start
        if (turret != null) {
            turret.initialize();
        }

        // Initialize ramp to 0 degrees at start
        if (ramp != null) {
            ramp.initialize();
            ElapsedTime initTimer = new ElapsedTime();
            while (!ramp.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                ramp.update();
                sleep(20);
            }
        }

        loopTimer.reset();
        telemetryTimer.reset();

        while (opModeIsActive()) {
            // Track loop time
            loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            avgLoopTimeMs = avgLoopTimeMs * 0.95 + loopTimeMs * 0.05;

            // === FAST UPDATES (every loop) ===
            // David's controls (gamepad1)
            handleDriveControls();
            handleDavidControls();

            // Edward's controls (gamepad2)
            handleEdwardControls();

            // Check uptake ball detection switch
            checkUptakeSwitch();

            // Update all controllers (PID loops run here)
            updateAllSystems();

            // === SLOW UPDATES (telemetry only every TELEMETRY_INTERVAL_MS) ===
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                updateTelemetry();
            }
        }
    }

    public void initializeHardware() {
        // Initialize drive controller (velocity-based with odometry)
        try {
            drive = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addLine("WARNING: Drive controller failed to initialize");
        }

        // Initialize turret (use 'this' for LinearOpMode constructor)
        try {
            turret = new TurretController(this);
        } catch (Exception ignored) {
        }

        // Initialize ramp
        try {
            ramp = new NewRampController(this);
        } catch (Exception ignored) {
        }

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception ignored) {
        }
        intake = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception ignored) {
        }
        transfer = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
        } catch (Exception ignored) {
        }
        uptake = new UptakeController(uptakeServo);

        // Initialize uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception ignored) {
        }

        // Initialize shooter (fixed motor names: "shooter" not "shooter1")
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
        } catch (Exception ignored) {
        }
        try {
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception ignored) {
        }
        shooter = new NewShooterController(shooterMotor1, shooterMotor2);
    }

    /**
     * Drive controls - David (gamepad1)
     * Left stick Y = forward/backward, Right stick X = rotation
     * Uses velocity-based control via SixWheelDriveController
     */
    public void handleDriveControls() {
        if (drive == null) return;

        // Update odometry
        drive.updateOdometry();

        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        // Get speed multipliers based on current mode
        double speedMult = drive.isFastSpeedMode() ?
                SixWheelDriveController.FAST_SPEED_MULTIPLIER :
                SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turnMult = drive.isFastSpeedMode() ?
                SixWheelDriveController.FAST_TURN_MULTIPLIER :
                SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        // Apply sensitivity curve and multipliers
        double drive_power = applySensitivityCurve(rawDrive) * speedMult;
        double turn_power = applySensitivityCurve(rawTurn) * turnMult;

        // Use arcade drive (handles velocity control internally)
        drive.arcadeDrive(drive_power, turn_power);
    }

    private double applySensitivityCurve(double input) {
        // Cubic curve for fine control at low speeds
        return input * input * input;
    }

    /**
     * David's controls (gamepad1)
     */
    public void handleDavidControls() {
        // Right bumper - toggle intake mode
        boolean currentRB1 = gamepad1.right_bumper;
        if (currentRB1 && !lastRightBumper1) {
            if (!intakeModeActive) {
                startIntakeMode();
            } else {
                stopIntakeMode();
            }
        }
        lastRightBumper1 = currentRB1;

        // Left bumper - toggle drive speed mode
        boolean currentA1 = gamepad1.a;
        if (currentA1 && !lastA1) {
            if (drive != null) {
                drive.toggleSpeedMode();
            }
        }
        lastA1 = currentA1;

        // X button - vomit mode toggle
        boolean currentX1 = gamepad1.x;
        if (currentX1 && !lastX1) {
            if (!vomitModeActive) {
                startVomitMode();
            } else {
                stopVomitMode();
            }
        }
        lastX1 = currentX1;

        // Y button - turret manual right
        if (gamepad1.y && turret != null) {
            turret.setPower(0.5);
        } else if (gamepad1.b && turret != null) {
            turret.setPower(-0.5);
        } else {
            turret.stop();
        }
    }

    private void startIntakeMode() {
        intakeModeActive = true;
        vomitModeActive = false;
        uptakeStoppedBySwitch = false;

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

    private void stopIntakeMode() {
        intakeModeActive = false;
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    private void startVomitMode() {
        vomitModeActive = true;
        intakeModeActive = false;

        if (intake != null) {
            intake.reversed = true;
            if (!intake.isActive()) intake.toggle();
        }
        if (transfer != null) {
            transfer.reversed = true;
            if (!transfer.isActive()) transfer.toggle();
        }
        if (uptake != null) {
            uptake.reversed = true;
            if (!uptake.isActive()) uptake.toggle();
        }
    }

    private void stopVomitMode() {
        vomitModeActive = false;
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    /**
     * Edward's controls (gamepad2)
     */
    public void handleEdwardControls() {
        // X button - far preset
        boolean currentX2 = gamepad2.x;
        if (currentX2 && !lastX2) {
            if (shooter != null) {
                shooter.setTargetRPM(FAR_RPM);
                currentTargetRPM = FAR_RPM;
                if (!shooter.isShootMode()) shooter.toggleShoot();
            }
            if (ramp != null) ramp.setTargetAngle(FAR_RAMP_ANGLE);
        }
        lastX2 = currentX2;

        // A button - close preset
        boolean currentA2 = gamepad2.a;
        if (currentA2 && !lastA2) {
            if (shooter != null) {
                shooter.setTargetRPM(CLOSE_RPM);
                currentTargetRPM = CLOSE_RPM;
                if (!shooter.isShootMode()) shooter.toggleShoot();
            }
            if (ramp != null) ramp.setTargetAngle(CLOSE_RAMP_ANGLE);
        }
        lastA2 = currentA2;

        // Y button - increase ramp angle
        boolean currentY2 = gamepad2.y;
        if (currentY2 && !lastY2) {
            if (ramp != null) ramp.incrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastY2 = currentY2;

        // B button - decrease ramp angle
        boolean currentB2 = gamepad2.b;
        if (currentB2 && !lastB2) {
            if (ramp != null) ramp.decrementAngle(RAMP_INCREMENT_DEGREES);
        }
        lastB2 = currentB2;

        // Dpad Up - increase target RPM
        boolean currentDpadUp2 = gamepad2.dpad_up;
        if (currentDpadUp2 && !lastDpadUp2) {
            if (shooter != null) {
                currentTargetRPM += RPM_INCREMENT;
                if (currentTargetRPM > NewShooterController.MAX_RPM) {
                    currentTargetRPM = NewShooterController.MAX_RPM;
                }
                shooter.setTargetRPM(currentTargetRPM);
            }
        }
        lastDpadUp2 = currentDpadUp2;

        // Dpad Down - decrease target RPM
        boolean currentDpadDown2 = gamepad2.dpad_down;
        if (currentDpadDown2 && !lastDpadDown2) {
            if (shooter != null) {
                currentTargetRPM -= RPM_INCREMENT;
                if (currentTargetRPM < NewShooterController.MIN_RPM) {
                    currentTargetRPM = NewShooterController.MIN_RPM;
                }
                shooter.setTargetRPM(currentTargetRPM);
            }
        }
        lastDpadDown2 = currentDpadDown2;

        // Right bumper - shooting mode toggle
        boolean currentRB2 = gamepad2.right_bumper;
        if (currentRB2 && !lastRightBumper2) {
            if (!isShooting) {
                // Start shooting sequence
                isShooting = true;
                intakeModeActive = false;
                vomitModeActive = false;
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
            } else {
                // Stop shooting sequence
                isShooting = false;
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
                if (shooter != null) {
                    shooter.stopShooting();
                    currentTargetRPM = 0;
                }
            }
        }
        lastRightBumper2 = currentRB2;

        // Left bumper - start/stop alignment
        boolean currentLB2 = gamepad2.left_bumper;
        if (currentLB2 && !lastLeftBumper2) {
        }
        lastLeftBumper2 = currentLB2;

        // Right trigger - hold for uptake
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            if (uptake != null && !uptake.isActive()) {
                uptake.reversed = false;
                uptake.toggle();
                uptakeFromTrigger = true;
            }
        } else {
            if (uptakeFromTrigger && uptake != null && uptake.isActive()) {
                uptake.toggle();
                uptakeFromTrigger = false;
            }
        }

        // Left trigger - stop intake system
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            stopIntakeSystem();
        }
    }

    /**
     * Stops intake, transfer, and uptake completely.
     * Starts shooter at idle RPM.
     */
    private void stopIntakeSystem() {
        intakeModeActive = false;
        vomitModeActive = false;
        isShooting = false;
        uptakeStoppedBySwitch = false;
        uptakeFromTrigger = false;

        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();

        if (shooter != null) {
            shooter.setTargetRPM(IDLE_RPM);
            currentTargetRPM = IDLE_RPM;
            if (!shooter.isShootMode()) {
                shooter.toggleShoot();
            }
        }
    }

    /**
     * Check uptake switch for ball detection
     */
    public void checkUptakeSwitch() {
        if (uptakeSwitch == null || uptake == null || isShooting) return;

        boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

        if (intakeModeActive) {
            if (ballDetected && !uptakeStoppedBySwitch) {
                if (uptake.isActive()) uptake.toggle();
                uptakeStoppedBySwitch = true;
            } else if (!ballDetected && uptakeStoppedBySwitch) {
                if (!uptake.isActive()) {
                    uptake.reversed = false;
                    uptake.toggle();
                }
                uptakeStoppedBySwitch = false;
            }
        } else {
            uptakeStoppedBySwitch = false;
        }
    }

    /**
     * Update all subsystem controllers (PID loops)
     */
    private void updateAllSystems() {
        if (turret != null) turret.update();  // Added: was missing!
        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }

    /**
     * Update telemetry (called at reduced rate for performance)
     */
    private void updateTelemetry() {
        // Status line
        String intakeStatus = intakeModeActive ? "INTAKE" : (vomitModeActive ? "VOMIT" : (isShooting ? "SHOOT" : "OFF"));
        String shooterStatus = shooter != null && shooter.isShootMode() ?
                (shooter.isAtTargetRPM() ? "READY" : "SPIN") : "OFF";
        String speedMode = (drive != null && drive.isFastSpeedMode()) ? "FAST" : "SLOW";

        telemetry.addData("Status", "%s | Shooter: %s | %s", intakeStatus, shooterStatus, speedMode);
        telemetry.addData("Loop", "%.1fms (%.0fHz)", avgLoopTimeMs, 1000.0 / avgLoopTimeMs);
        telemetry.addLine();

        // Drive info
        if (drive != null) {
            telemetry.addData("Position", "X:%.1f Y:%.1f", drive.getX(), drive.getY());
            telemetry.addData("Heading", "%.1f", drive.getHeadingDegrees());
        }

        // Shooter info
        if (shooter != null) {
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
            telemetry.addData("RPM Error", "%.0f", shooter.getRPMError());
        }

        // Ramp info
        if (ramp != null) {
            telemetry.addData("Ramp", "%.1f", ramp.getCurrentAngle());
        }

        // Turret info
        if (turret != null) {
            telemetry.addData("Turret", "%.1f", turret.getTurretAngle());
        }

        // Ball detection
        if (uptakeSwitch != null) {
            boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
            telemetry.addData("Ball", ballDetected ? "DETECTED" : "none");
        }
        telemetry.update();
    }

    private double getBatteryVoltage() {
        double voltage = 0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }
        return voltage;
    }
}