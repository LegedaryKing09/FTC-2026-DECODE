package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@TeleOp(name = "December Teleop", group = "Competition")
public class DecemberTeleop extends LinearOpMode {

    // Ramp angle increment (tunable via FTC Dashboard)
    public static double RAMP_INCREMENT_DEGREES = 10.0;

    // Turret control sensitivity
    public static double TURRET_SENSITIVITY = 3.0;

    // Target AprilTag ID for turret alignment
    public static int TURRET_TARGET_TAG_ID = 20;

    // Shooter presets (tunable via FTC Dashboard)
    public static double CLOSE_RPM = 3650.0;
    public static double FAR_RPM = 4600.0;
    public static double SHOOTER_RPM = 4800.0;

    // RPM manual adjustment increment (tunable via FTC Dashboard)
    public static double RPM_INCREMENT = 50.0;

    // Ramp angle presets (adjust these based on testing)
    public static double CLOSE_RAMP_ANGLE = 171.0;
    public static double FAR_RAMP_ANGLE = 92.0;

    // Controllers
    private TurretController turret;
    private TurretAlignmentController turretAlignment;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;
    private NewShooterController shooter;
    private NewRampController ramp;

    // Drive motors
    private DcMotor lf, lb;
    private DcMotor rf, rb;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;  // Voltage threshold for ball detection

    private final ElapsedTime runtime = new ElapsedTime();

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

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Intake mode state (all wheels together)
    private boolean intakeModeActive = false;

    // Vomit mode state (reverse all wheels)
    private boolean vomitModeActive = false;

    // Track if uptake was stopped by ball detection switch
    private boolean uptakeStoppedBySwitch = false;

    // Track if shooter is running from left trigger
    private boolean shooterFromTrigger = false;

    // Track current shooter target RPM for display
    private double currentTargetRPM = 0;

    // Track if uptake is running from Edward's right trigger
    private boolean uptakeFromTrigger = false;

    private boolean isShooting = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        initializeHardware();
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize ramp to 0 degrees at start
        if (ramp != null) {
            ramp.initialize();
            ElapsedTime initTimer = new ElapsedTime();
            while (!ramp.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                ramp.update();
                sleep(20);
            }
        }

        while (opModeIsActive()) {
            // David's controls (gamepad1)
            handleDriveControls();
            handleDavidControls();

            // Edward's controls (gamepad2)
            handleEdwardControls();

            // Check uptake ball detection switch
            checkUptakeSwitch();

            // Update all controllers
            updateAllSystems();

            // Display telemetry
            displayTelemetry();
        }
    }

    public void initializeHardware() {
        // Initialize drive motors
        try {
            lf = hardwareMap.get(DcMotor.class, "lf");
            lb = hardwareMap.get(DcMotor.class, "lb");
            rf = hardwareMap.get(DcMotor.class, "rf");
            rb = hardwareMap.get(DcMotor.class, "rb");

            lf.setDirection(DcMotorSimple.Direction.FORWARD);
            lb.setDirection(DcMotorSimple.Direction.FORWARD);
            rf.setDirection(DcMotorSimple.Direction.REVERSE);
            rb.setDirection(DcMotorSimple.Direction.REVERSE);

            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("✓ Drive", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Drive", "NOT FOUND");
        }

        // Initialize turret
        try {
            turret = new TurretController(this);
            telemetry.addData("✓ Turret", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Turret", "NOT FOUND: " + e.getMessage());
        }

        // Initialize turret alignment controller
        try {
            turretAlignment = new TurretAlignmentController(this, turret);
            turretAlignment.setTargetTag(TURRET_TARGET_TAG_ID);
            telemetry.addData("✓ Turret Alignment", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Turret Alignment", "NOT FOUND: " + e.getMessage());
        }

        // Initialize ramp
        try {
            ramp = new NewRampController(this);
            telemetry.addData("✓ Ramp", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Ramp", "NOT FOUND: " + e.getMessage());
        }

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            telemetry.addData("✓ Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Intake", "NOT FOUND");
        }
        intake = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            telemetry.addData("✓ Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Transfer", "NOT FOUND");
        }
        transfer = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            telemetry.addData("✓ Uptake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Uptake", "NOT FOUND");
        }
        uptake = new UptakeController(uptakeServo);

        // Initialize uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
            telemetry.addData("✓ Uptake Switch", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Uptake Switch", "NOT FOUND");
        }

        // Initialize shooter
        DcMotor shooterMotor = null;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            telemetry.addData("✓ Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter", "NOT FOUND");
        }
        shooter = new NewShooterController(shooterMotor);
    }

    /**
     * Drive controls - David (gamepad1)
     * Left stick Y = forward/backward, Right stick X = rotation
     */
    public void handleDriveControls() {
        double rawDrive = -gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        double drive = applySensitivityCurve(rawDrive);
        double turn = applySensitivityCurve(rawTurn);

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        if (lf != null) lf.setPower(leftPower);
        if (lb != null) lb.setPower(leftPower);
        if (rf != null) rf.setPower(rightPower);
        if (rb != null) rb.setPower(rightPower);
    }

    private double applySensitivityCurve(double value) {
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double curved = Math.pow(magnitude, 2.0);
        return sign * curved;
    }

    /**
     * David's controls (gamepad1):
     * Right bumper - toggle intake mode (intake + transfer + uptake)
     * Right trigger - hold for vomit mode (reverse all)
     * X - toggle intake only
     * Y - toggle transfer only
     * A - toggle uptake only
     */
    private void handleDavidControls() {
        // Right bumper - toggle intake mode (all wheels together)
        boolean currentRB1 = gamepad1.right_bumper;
        if (currentRB1 && !lastRightBumper1) {
            isShooting = false;
            intakeModeActive = !intakeModeActive;
            vomitModeActive = false; // Disable vomit when toggling intake mode

            if (intakeModeActive) {
                if (intake != null) { intake.reversed = false; if (!intake.isActive()) intake.toggle(); }
                if (transfer != null) { transfer.reversed = false; if (!transfer.isActive()) transfer.toggle(); }
                if (uptake != null) { uptake.reversed = false; if (!uptake.isActive()) uptake.toggle(); }
            } else {
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
            }
        }
        lastRightBumper1 = currentRB1;

        // Right trigger - hold for vomit mode (press = on, release = off)
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (!vomitModeActive) {
                vomitModeActive = true;
                intakeModeActive = false;
                if (intake != null) { intake.reversed = true; if (!intake.isActive()) intake.toggle(); }
                if (transfer != null) { transfer.reversed = true; if (!transfer.isActive()) transfer.toggle(); }
                if (uptake != null) { uptake.reversed = true; if (!uptake.isActive()) uptake.toggle(); }
            }
        } else {
            if (vomitModeActive) {
                vomitModeActive = false;
                if (intake != null && intake.isActive()) intake.toggle();
                if (transfer != null && transfer.isActive()) transfer.toggle();
                if (uptake != null && uptake.isActive()) uptake.toggle();
            }
        }

        // X button - toggle intake only
        boolean currentX1 = gamepad1.x;
        if (currentX1 && !lastX1) {
            if (intake != null) {
                intake.reversed = false;
                intake.toggle();
            }
        }
        lastX1 = currentX1;

        // Y button - toggle transfer only
        boolean currentY1 = gamepad1.y;
        if (currentY1 && !lastY1) {
            if (transfer != null) {
                transfer.reversed = false;
                transfer.toggle();
            }
        }
        lastY1 = currentY1;

        // A button - toggle uptake only
        boolean currentA1 = gamepad1.a;
        if (currentA1 && !lastA1) {
            if (uptake != null) {
                uptake.reversed = false;
                uptake.toggle();
            }
        }
        lastA1 = currentA1;
    }

    /**
     * Edward's controls (gamepad2):
     * Left stick X - turret rotation
     * X - far preset (RPM + ramp)
     * A - close preset (RPM + ramp)
     * Y - increase ramp angle
     * B - decrease ramp angle
     * Dpad Up - increase target RPM
     * Dpad Down - decrease target RPM
     * Right bumper - (placeholder for turret auto shoot toggle)
     * Right trigger - hold for uptake
     * Left trigger - run shooter at 4800 RPM
     */
    private void handleEdwardControls() {
        // Left stick X - turret control
        if (turret != null) {
            double turretInput = gamepad2.left_stick_x;
            if (Math.abs(turretInput) != 0) {
                if (turret.getCurrentAngle() <= 360 && turret.getCurrentAngle() >= 0) {
                    turret.setPower(turretInput);
                }
            }

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

            // Right bumper
            boolean currentRB2 = gamepad2.right_bumper;
            if (currentRB2 && !lastRightBumper2) {
                isShooting = true;
                intakeModeActive = false;
                vomitModeActive = false;
                intake.toggle();
                transfer.toggle();
                uptake.toggle();
            }
            lastRightBumper2 = currentRB2;

            // Left bumper - toggle shooter at current target RPM (set by d-pad)
            boolean currentLB2 = gamepad2.left_bumper;
            if (currentLB2 && !lastLeftBumper2) {
                turretAlignment.startAlignment();
            }
            lastLeftBumper2 = currentLB2;

            // Right trigger - hold for uptake (press = on, release = off)
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

            // Left trigger - hold to run shooter at 4800 RPM
            if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                if (shooter != null) {
                    shooter.setTargetRPM(SHOOTER_RPM);
                    currentTargetRPM = SHOOTER_RPM;
                    if (!shooter.isShootMode()) shooter.toggleShoot();
                    shooterFromTrigger = true;
                }
            } else if (shooterFromTrigger) {
                if (shooter != null) shooter.stopShooting();
                shooterFromTrigger = false;
                currentTargetRPM = 0;
            }
        }
    }

    /**
     * Check uptake switch for ball detection
     * When in intake mode and ball detected, stop uptake but keep intake/transfer running
     * Switch is pressed down (ball present) when voltage drops BELOW threshold
     */
    public void checkUptakeSwitch() {
        if (uptakeSwitch == null || uptake == null || isShooting) return;

        // FIXED: Ball detected when voltage is BELOW threshold (switch pressed down)
        boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

        // Only auto-stop uptake during intake mode
        if (intakeModeActive) {
            if (ballDetected && !uptakeStoppedBySwitch) {
                // Ball detected - stop uptake, keep intake and transfer running
                if (uptake.isActive()) uptake.toggle();
                uptakeStoppedBySwitch = true;
            } else if (!ballDetected && uptakeStoppedBySwitch) {
                // Ball removed - restart uptake if still in intake mode
                if (!uptake.isActive()) {
                    uptake.reversed = false;
                    uptake.toggle();
                }
                uptakeStoppedBySwitch = false;
            }
        } else {
            // Reset flag when not in intake mode
            uptakeStoppedBySwitch = false;
        }
    }

    private void updateAllSystems() {
        if (ramp != null) ramp.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
        if (shooter != null) shooter.update();
    }

    private void displayTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());

        // Drive
        telemetry.addLine("═══ DRIVE (David) ═══");
        double leftPwr = lf != null ? lf.getPower() : 0;
        double rightPwr = rf != null ? rf.getPower() : 0;
        telemetry.addData("L/R Power", "%.2f / %.2f", leftPwr, rightPwr);

        // Intake system status
        telemetry.addLine("═══ INTAKE SYSTEM ═══");
        telemetry.addData("Mode", intakeModeActive ? "INTAKE" : (vomitModeActive ? "VOMIT" : "MANUAL"));
        telemetry.addData("Intake", (intake != null && intake.isActive()) ? "ON" : "OFF");
        telemetry.addData("Transfer", (transfer != null && transfer.isActive()) ? "ON" : "OFF");
        telemetry.addData("Uptake", (uptake != null && uptake.isActive()) ? "ON" : "OFF");
        if (uptakeSwitch != null) {
            boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
            telemetry.addData("Ball Detected", ballDetected ? "YES" : "NO");
        }

        // Turret
        if (turret != null) {
            telemetry.addLine("═══ TURRET (Edward) ═══");
            telemetry.addData("Angle", "%.1f°", turret.getCurrentAngle());
        }

        // Ramp
        if (ramp != null) {
            telemetry.addLine("═══ RAMP ═══");
            telemetry.addData("Angle", "%.1f°", ramp.getCurrentAngle());
            telemetry.addData("Target", "%.1f°", ramp.getTargetAngle());
        }

        // Shooter
        if (shooter != null) {
            telemetry.addLine("═══ SHOOTER ═══");
            telemetry.addData("RPM", "%.0f / %.0f", shooter.getRPM(), currentTargetRPM);
            telemetry.addData("Mode", shooter.isShootMode() ? "SHOOTING" : "IDLE");
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