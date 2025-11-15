package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@TeleOp(name = "TeleOp V4", group = "Competition")
public class AutoTeleop extends LinearOpMode {

    // Controllers
    private SixWheelDriveController driveController;
    private TransferController transferController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private LimelightAlignmentController limelightController;
    private AutoShootController autoShootController;
    private RampController rampController;

    // Shooter Management (Simplified to 3 levels)
    public static double IDLE_RPM = 1800;      // Higher idle for faster response
    public static double CLOSE_RPM = 2800;
    public static double FAR_RPM = 3100;

    // Battery thresholds
    public static double LOW_VOLTAGE = 11.0;

    // Current states
    private double currentTargetRPM = IDLE_RPM;
    private boolean shooterActive = false;
    private boolean intakeRunning = false;
    private boolean isAligning = false;

    // Timing
    private ElapsedTime runtimeTimer = new ElapsedTime();
    private ElapsedTime activityTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();

    // Single background thread for PID
    private Thread pidThread;
    private volatile boolean runPid = true;

    // Battery monitoring
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;

    // Button tracking (simplified)
    private boolean lastA = false, lastB = false, lastY = false, lastX = false;
    private boolean lastRB = false, lastLB = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtimeTimer.reset();

        // Start single PID thread
        startPidThread();

        while (opModeIsActive()) {
            // Drive control
            handleDriving();

            // Button control
            handleButtons();

            // Shooter management (simple timeout)
            manageShooter();

            // Update telemetry sparingly (every 500ms)
            if (telemetryTimer.milliseconds() > 500) {
                updateTelemetry();
                telemetryTimer.reset();
            }
        }

        cleanup();
    }

    private void initialize() {
        // Initialize controllers
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);

        // Try to get voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Initialize vision (optional)
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new AutoShootController(
                    this, driveController, shooterController, intakeController,
                    transferController, limelightController, rampController
            );
        } catch (Exception e) {
            // Continue without vision
        }

        // Set initial state
        rampController.setTo0Degrees();
        driveController.setFastSpeed();
        shooterController.setShooterRPM(IDLE_RPM);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    private void startPidThread() {
        pidThread = new Thread(() -> {
            while (runPid && opModeIsActive()) {
                shooterController.updatePID();

                // Check voltage occasionally
                if (voltageSensor != null && runtimeTimer.seconds() % 2 < 0.05) {
                    voltage = voltageSensor.getVoltage();
                }

                try {
                    Thread.sleep(20); // 50Hz
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        pidThread.setPriority(Thread.MAX_PRIORITY);
        pidThread.start();
    }

    private void handleDriving() {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // Speed multipliers
        if (driveController.isFastSpeedMode()) {
            drive *= SixWheelDriveController.FAST_SPEED_MULTIPLIER;
            turn *= SixWheelDriveController.FAST_TURN_MULTIPLIER;
        } else {
            drive *= SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
            turn *= SixWheelDriveController.SLOW_TURN_MULTIPLIER;
        }

        // Low battery mode
        if (voltage < LOW_VOLTAGE) {
            drive *= 0.8;
            turn *= 0.8;
        }

        driveController.arcadeDrive(drive, turn);
    }

    private void handleButtons() {
        // A: Close shot
        if (gamepad1.a && !lastA) {
            setShooterPreset(CLOSE_RPM, 0.71);
            shooterActive = true;
            activityTimer.reset();
        }
        lastA = gamepad1.a;

        // B: Far shot
        if (gamepad1.b && !lastB) {
            setShooterPreset(FAR_RPM, 0.70);
            shooterActive = true;
            activityTimer.reset();
        }
        lastB = gamepad1.b;

        // Y: Medium shot
        if (gamepad1.y && !lastY) {
            setShooterPreset(2950, 0.705);
            shooterActive = true;
            activityTimer.reset();
        }
        lastY = gamepad1.y;

        // X: Emergency stop
        if (gamepad1.x && !lastX) {
            emergencyStop();
        }
        lastX = gamepad1.x;

        // Right trigger: Transfer (hold)
        if (gamepad1.right_trigger > 0.1) {
            transferController.transferFull();
            // Ensure shooter is active when transferring
            if (!shooterActive && currentTargetRPM > IDLE_RPM) {
                shooterActive = true;
                activityTimer.reset();
            }
        } else {
            transferController.transferStop();
        }

        // Right bumper: Toggle intake
        if (gamepad1.right_bumper && !lastRB) {
            if (intakeRunning) {
                intakeController.intakeStop();
                intakeRunning = false;
            } else {
                intakeController.intakeFull();
                intakeRunning = true;
                // Pre-spin shooter when intaking
                if (currentTargetRPM == IDLE_RPM) {
                    shooterController.setShooterRPM(CLOSE_RPM);
                    currentTargetRPM = CLOSE_RPM;
                }
                activityTimer.reset();
            }
        }
        lastRB = gamepad1.right_bumper;

        // Left bumper: Eject
        if (gamepad1.left_bumper && !lastLB) {
            if (intakeRunning) {
                intakeController.intakeStop();
                intakeRunning = false;
            } else {
                intakeController.intakeEject();
            }
        } else if (!gamepad1.left_bumper && lastLB) {
            if (!intakeRunning) {
                intakeController.intakeStop();
            }
        }
        lastLB = gamepad1.left_bumper;

        // D-pad left: Auto-shoot
        if (gamepad1.dpad_left && !lastDpadLeft) {
            if (autoShootController != null && autoShootController.isNotAutoShooting()) {
                // Ensure shooter is spun up first
                if (currentTargetRPM == IDLE_RPM) {
                    shooterController.setShooterRPM(CLOSE_RPM);
                    currentTargetRPM = CLOSE_RPM;
                }
                // Use non-adjusting mode for speed
                autoShootController.executeDistanceBasedAutoShoot(false, false, false);
                activityTimer.reset();
            }
        }
        lastDpadLeft = gamepad1.dpad_left;

        // D-pad right: Toggle alignment
        if (gamepad1.dpad_right && !lastDpadRight) {
            if (limelightController != null) {
                if (!isAligning) {
                    isAligning = true;
                    limelightController.startAlignment();
                } else {
                    isAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }
        }
        lastDpadRight = gamepad1.dpad_right;

        // Handle alignment
        if (isAligning && limelightController != null) {
            limelightController.align(AutoShootController.APRILTAG_ID);

            // Auto-stop after 1.5 seconds
            if (activityTimer.seconds() > 1.5) {
                isAligning = false;
                limelightController.stopAlignment();
                driveController.stopDrive();
            }
        }

        // Back button: Speed toggle
        if (gamepad1.back) {
            if (driveController.isFastSpeedMode()) {
                driveController.setSlowSpeed();
            } else {
                driveController.setFastSpeed();
            }
        }
    }

    private void manageShooter() {
        // Simple timeout-based idle return
        if (shooterActive && activityTimer.seconds() > 3.0) {
            // Return to idle after 3 seconds of inactivity
            shooterController.setShooterRPM(IDLE_RPM);
            currentTargetRPM = IDLE_RPM;
            shooterActive = false;
        }
    }

    private void setShooterPreset(double rpm, double rampPos) {
        shooterController.setShooterRPM(rpm);
        currentTargetRPM = rpm;
        rampController.setPosition(rampPos);
    }

    private void emergencyStop() {
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        currentTargetRPM = 0;
        shooterActive = false;
        intakeRunning = false;
        isAligning = false;
    }

    private void updateTelemetry() {
        telemetry.clear();

        // Essential info only
        telemetry.addData("Shooter", "%.0f/%.0f RPM",
                shooterController.getShooterRPM(), currentTargetRPM);

        if (voltageSensor != null) {
            telemetry.addData("Battery", "%.1fV %s",
                    voltage, voltage < LOW_VOLTAGE ? "LOW" : "OK");
        }

        telemetry.addData("Mode", driveController.isFastSpeedMode() ? "FAST" : "SLOW");
        telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");

        if (isAligning && limelightController != null) {
            telemetry.addData("Align", "%.1f°", limelightController.getTargetError());
        }

        telemetry.addData("Runtime", "%.0fs", runtimeTimer.seconds());

        telemetry.update();
    }

    private void cleanup() {
        runPid = false;

        if (pidThread != null) {
            try {
                pidThread.interrupt();
                pidThread.join(100);
            } catch (Exception e) {
                // Ignore
            }
        }

        emergencyStop();
    }
}