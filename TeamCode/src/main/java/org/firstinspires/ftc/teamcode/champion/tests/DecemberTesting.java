package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;

/**
 * Testing teleop using newer controllers with BasicTeleop-style manual controls
 * Controls (single gamepad):
 * - Left Stick Y: Drive forward/backward
 * - Right Stick X: Turn
 * - A: Decrease ramp angle
 * - B: Increase ramp angle
 * - Y: Start shooter at current target RPM
 * - X: Stop all (shooter, intake, transfer, uptake)
 * - Dpad Up: Increase target RPM
 * - Dpad Down: Decrease target RPM
 * - Right Trigger: Run transfer (hold)
 * - Right Bumper: Run intake (hold)
 * - Left Bumper: Reverse intake/eject (hold)
 * - Left Trigger: Run uptake (hold)
 * - Back: Toggle fast/slow drive speed
 * - Start: Toggle telemetry display
 */
@Config
@TeleOp
public class DecemberTesting extends LinearOpMode {

    // Tunable parameters via FTC Dashboard
    public static double TARGET_RPM = 3000.0;
    public static double RPM_INCREMENT = 50.0;
    public static double RAMP_INCREMENT = 2.5;

    // Drive speed multipliers
    public static double SLOW_SPEED = 0.5;
    public static double FAST_SPEED = 1.0;
    public static double SLOW_TURN = 0.4;
    public static double FAST_TURN = 0.8;

    // Controllers
    private NewShooterController shooter;
    private NewRampController ramp;
    private NewIntakeController intake;
    private NewTransferController transfer;
    private UptakeController uptake;

    // Drive motors
    private DcMotor lf, lb, rf, rb;

    private final ElapsedTime runtime = new ElapsedTime();

    // Button state tracking (for edge detection)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastBack = false;
    private boolean lastStart = false;

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.1;

    // State variables
    private boolean isFastMode = false;
    private boolean showTelemetry = true;

    @Override
    public void runOpMode() {
        // Setup telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initializeHardware();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize ramp to 0 degrees
        if (ramp != null) {
            ramp.initialize();
            ElapsedTime initTimer = new ElapsedTime();
            while (!ramp.isInitialized() && initTimer.seconds() < 3.0 && opModeIsActive()) {
                ramp.update();
                sleep(20);
            }
        }

        // Set initial target RPM
        if (shooter != null) {
            shooter.setTargetRPM(TARGET_RPM);
        }

        while (opModeIsActive()) {
            // Handle all controls
            handleDriveControls();
            handleShooterControls();
            handleRampControls();
            handleIntakeControls();
            handleMiscControls();

            // Update all controllers
            updateAllSystems();

            // Display telemetry
            if (showTelemetry) {
                displayTelemetry();
            } else {
                telemetry.addData("Telemetry", "DISABLED (Press Start to enable)");
                telemetry.update();
            }
        }
    }

    private void initializeHardware() {
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
            telemetry.addData("✗ Drive", "NOT FOUND: " + e.getMessage());
        }

        // Initialize shooter
        try {
            DcMotor shooterMotorFirst = hardwareMap.get(DcMotor.class, "shooter");
            DcMotor shooterMotorSecond = hardwareMap.get(DcMotor.class, "shooter2");
            shooter = new NewShooterController(shooterMotorFirst);
            telemetry.addData("✓ Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter", "NOT FOUND: " + e.getMessage());
        }

        // Initialize ramp
        try {
            ramp = new NewRampController(this);
            telemetry.addData("✓ Ramp", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Ramp", "NOT FOUND: " + e.getMessage());
        }

        // Initialize intake
        try {
            DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intake = new NewIntakeController(intakeMotor);
            telemetry.addData("✓ Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Intake", "NOT FOUND: " + e.getMessage());
        }

        // Initialize transfer
        try {
            DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            transfer = new NewTransferController(transferMotor);
            telemetry.addData("✓ Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Transfer", "NOT FOUND: " + e.getMessage());
        }

        // Initialize uptake
        CRServo uptakeServo = null;
        CRServo uptakeServo2 = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            uptakeServo2 = hardwareMap.get(CRServo.class, "uptake2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake: " + e.getMessage());
        }
        uptake = new UptakeController(uptakeServo, uptakeServo2);
    }

    /**
     * Arcade drive controls using left stick Y and right stick X
     */
    private void handleDriveControls() {
        double speedMultiplier = isFastMode ? FAST_SPEED : SLOW_SPEED;
        double turnMultiplier = isFastMode ? FAST_TURN : SLOW_TURN;

        double drive = -gamepad1.left_stick_y * speedMultiplier;
        double turn = gamepad1.right_stick_x * turnMultiplier;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers if exceeding 1.0
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

    /**
     * Shooter controls:
     * - Y: Start shooter at target RPM
     * - X: Stop shooter (and all other systems)
     * - Dpad Up/Down: Adjust target RPM
     */
    private void handleShooterControls() {
        // Y button: Start shooter
        if (gamepad1.y && !lastY) {
            if (shooter != null && !shooter.isShootMode()) {
                shooter.startShooting();
            }
        }
        lastY = gamepad1.y;

        // X button: Stop all systems
        if (gamepad1.x && !lastX) {
            stopAllSystems();
        }
        lastX = gamepad1.x;

        // Dpad Up: Increase RPM
        if (gamepad1.dpad_up && !lastDpadUp) {
            TARGET_RPM += RPM_INCREMENT;
            if (TARGET_RPM > NewShooterController.MAX_RPM) {
                TARGET_RPM = NewShooterController.MAX_RPM;
            }
            if (shooter != null) {
                shooter.setTargetRPM(TARGET_RPM);
            }
        }
        lastDpadUp = gamepad1.dpad_up;

        // Dpad Down: Decrease RPM
        if (gamepad1.dpad_down && !lastDpadDown) {
            TARGET_RPM -= RPM_INCREMENT;
            if (TARGET_RPM < NewShooterController.MIN_RPM) {
                TARGET_RPM = NewShooterController.MIN_RPM;
            }
            if (shooter != null) {
                shooter.setTargetRPM(TARGET_RPM);
            }
        }
        lastDpadDown = gamepad1.dpad_down;
    }

    /**
     * Ramp controls:
     * - A: Decrease ramp angle
     * - B: Increase ramp angle
     */
    private void handleRampControls() {
        // A button: Decrease ramp angle
        if (gamepad1.a && !lastA) {
            if (ramp != null) {
                ramp.decrementAngle(RAMP_INCREMENT);
            }
        }
        lastA = gamepad1.a;

        // B button: Increase ramp angle
        if (gamepad1.b && !lastB) {
            if (ramp != null) {
                ramp.incrementAngle(RAMP_INCREMENT);
            }
        }
        lastB = gamepad1.b;
    }

    /**
     * Intake system controls:
     * - Right Trigger: Transfer (hold)
     * - Right Bumper: Intake (hold)
     * - Left Bumper: Reverse intake/eject (hold)
     * - Left Trigger: Uptake (hold)
     */
    private void handleIntakeControls() {
        // Right Bumper: Intake (hold to run, release to stop)
        if (gamepad1.right_bumper) {
            if (intake != null) {
                intake.reversed = false;
                if (!intake.isActive()) intake.toggle();
            }
        } else {
            if (intake != null && intake.isActive() && !intake.reversed) {
                intake.toggle();
            }
        }

        // Left Bumper: Reverse intake/eject (hold to run, release to stop)
        if (gamepad1.left_bumper) {
            if (intake != null) {
                intake.reversed = true;
                if (!intake.isActive()) intake.toggle();
            }
        } else {
            if (intake != null && intake.isActive() && intake.reversed) {
                intake.toggle();
            }
        }

        // Right Trigger: Transfer (hold to run, release to stop)
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (transfer != null) {
                transfer.reversed = false;
                if (!transfer.isActive()) transfer.toggle();
            }
        } else {
            if (transfer != null && transfer.isActive()) {
                transfer.toggle();
            }
        }

        // Left Trigger: Uptake (hold to run, release to stop)
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            if (uptake != null) {
                uptake.reversed = false;
                if (!uptake.isActive()) uptake.toggle();
            }
        } else {
            if (uptake != null && uptake.isActive()) {
                uptake.toggle();
            }
        }
    }

    /**
     * Miscellaneous controls:
     * - Back: Toggle fast/slow drive mode
     * - Start: Toggle telemetry display
     */
    private void handleMiscControls() {
        // Back button: Toggle drive speed mode
        if (gamepad1.back && !lastBack) {
            isFastMode = !isFastMode;
        }
        lastBack = gamepad1.back;

        // Start button: Toggle telemetry
        if (gamepad1.start && !lastStart) {
            showTelemetry = !showTelemetry;
        }
        lastStart = gamepad1.start;
    }

    /**
     * Stop all active systems
     */
    private void stopAllSystems() {
        if (shooter != null) shooter.stopShooting();
        if (intake != null && intake.isActive()) intake.toggle();
        if (transfer != null && transfer.isActive()) transfer.toggle();
        if (uptake != null && uptake.isActive()) uptake.toggle();
    }

    /**
     * Update all controller states
     */
    private void updateAllSystems() {
        if (ramp != null) ramp.update();
        if (shooter != null) shooter.update();
        if (intake != null) intake.update();
        if (transfer != null) transfer.update();
        if (uptake != null) uptake.update();
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Drive Mode", isFastMode ? "FAST" : "SLOW");

        // Drive telemetry
        telemetry.addLine("═══ DRIVE ═══");
        double leftPwr = lf != null ? lf.getPower() : 0;
        double rightPwr = rf != null ? rf.getPower() : 0;
        telemetry.addData("L/R Power", "%.2f / %.2f", leftPwr, rightPwr);

        // Shooter telemetry
        telemetry.addLine("═══ SHOOTER ═══");
        if (shooter != null) {
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
            telemetry.addData("RPM Error", "%.0f", shooter.getRPMError());
            telemetry.addData("At Target", shooter.isAtTargetRPM() ? "✓ YES" : "NO");
            telemetry.addData("Mode", shooter.isShootMode() ? "SHOOTING" : "IDLE");
            telemetry.addData("Power", "%.2f", shooter.getCurrentPower());
        } else {
            telemetry.addData("Shooter", "NOT AVAILABLE");
        }

        // Ramp telemetry
        telemetry.addLine("═══ RAMP ═══");
        if (ramp != null) {
            telemetry.addData("Current Angle", "%.1f°", ramp.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", ramp.getTargetAngle());
            telemetry.addData("Error", "%.1f°", ramp.getAngleError());
            telemetry.addData("Moving", ramp.isMoving() ? "YES" : "NO");
        } else {
            telemetry.addData("Ramp", "NOT AVAILABLE");
        }

        // Intake system telemetry
        telemetry.addLine("═══ INTAKE SYSTEM ═══");
        telemetry.addData("Intake", (intake != null && intake.isActive()) ?
                (intake.reversed ? "REVERSE" : "ON") : "OFF");
        telemetry.addData("Transfer", (transfer != null && transfer.isActive()) ? "ON" : "OFF");
        telemetry.addData("Uptake", (uptake != null && uptake.isActive()) ? "ON" : "OFF");

        // Control hints
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("Y=Shoot | X=Stop All");
        telemetry.addLine("A/B=Ramp | DUp/DDown=RPM");
        telemetry.addLine("RB=Intake | LB=Eject");
        telemetry.addLine("RT=Transfer | LT=Uptake");

        telemetry.update();
    }
}