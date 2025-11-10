package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;
import org.firstinspires.ftc.teamcode.champion.controller.AutonController;

import java.util.Locale;

@Config
@Autonomous(name = "Basic Auton Fixed", group = "Competition")
public class BasicAuton extends LinearOpMode {
    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;
    AutonController autonController;

    // Autonomous parameters
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double CONSTANT_RAMP_ANGLE = 121.0;
    public static double BACKWARD_DISTANCE = 50.0;
    public static double BASE_FORWARD_DISTANCE = 0;
    public static double BASE_REPOSITION_DISTANCE = 0;
    public static double MOVEMENT_SPEED = 0.6;
    public static double INTAKE_SPEED = 0.4;

    // Pattern parameters
    public static double SCAN_ANGLE = -40.0;

    // Pattern-specific parameters (all-in-one)
    public static double[] FETCH_ANGLES = {44.0, 31.0, 26.0};  // PPG, PGP, GPP
    public static double[] EXTRA_DISTANCES = {80.0, 50.0, 20.0}; // PPG, PGP, GPP

    private final ElapsedTime globalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Fast initialization
        initializeRobot();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Start shooter and ramp
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);

        // Start continuous PID and intake threads
        autonController.startPidUpdateThread();
        autonController.startIntakeThread();

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        autonController.stopIntakeThread();
        autonController.stopPidUpdateThread();
        shooterController.shooterStop();
        intakeController.intakeStop();

        telemetry.addData("Total Time", String.format(Locale.US, "%.1fs", globalTimer.seconds()));
        telemetry.update();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);

        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Try to initialize Limelight (don't fail if not available)
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new AutoShootController(this, driveController, shooterController,
                    intakeController, transferController, limelightController, rampController);
        } catch (Exception e) {
            limelightController = null;
            autoShootController = null;
        }

        // Initialize movement controller
        autonController = new AutonController(
                this, driveController, transferController, shooterController,
                intakeController, limelightController, autoShootController
        );

        telemetry.addLine("=== AUTON READY ===");
        telemetry.update();
    }

    private void executeAutonomousSequence() {
        // PHASE 1: Warmup while moving backward
        Thread warmupThread = new Thread(() -> autonController.warmupShooter());
        warmupThread.start();

        autonController.moveRobot(-BACKWARD_DISTANCE, MOVEMENT_SPEED);

        // Wait for warmup to complete (if not already)
        try {
            warmupThread.join(100); // Max 100ms wait
        } catch (InterruptedException e) {
            // Continue anyway
        }

        // Shoot preloaded balls
        autonController.quickShoot();

        // PHASE 2: Turn and scan for pattern
        autonController.turnToHeading(SCAN_ANGLE);

        // Pattern detection with continuous PID updates
        int patternIndex = autonController.detectPattern();

        // PHASE 3: Fetch balls
        double fetchAngle = FETCH_ANGLES[patternIndex];
        double fetchDistance = BASE_FORWARD_DISTANCE + EXTRA_DISTANCES[patternIndex];

        autonController.turnToHeading(fetchAngle);

        // Move forward with intake running (via thread)
        autonController.setIntakePower(1.0);  // Start intake via thread
        autonController.moveRobot(fetchDistance, INTAKE_SPEED);

        // Brief intake time with PID-aware sleep
        autonController.sleepWithPid(400);
        autonController.setIntakePower(0.0);  // Stop intake via thread

        // PHASE 4: Quick reposition and shoot
        double repositionDistance = BASE_REPOSITION_DISTANCE + EXTRA_DISTANCES[patternIndex];
        autonController.moveRobot(-repositionDistance, MOVEMENT_SPEED);

        autonController.turnToHeading(0); // Return to original heading
        autonController.quickShoot();
    }
}