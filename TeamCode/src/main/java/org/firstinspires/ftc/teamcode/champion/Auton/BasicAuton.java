package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.AutonController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

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

    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double CONSTANT_RAMP_ANGLE = 121.0;

    public static double MOVEMENT_SPEED = 0.4;
    public static double INTAKE_SPEED = 0.2;

    // ============ PATH PARAMETERS ============
    public static double INITIAL_BACKWARD = 50.0;
    public static double PATTERN_SCAN_ANGLE = -40.0;  // Turn right to face AprilTag directly
    public static double[] PATTERN_POSITION_DISTANCE = {
            -32.5,  // PPG
            -11.0,  // PGP
            2.0     // GPP
    };
    public static double LEFT_TURN_ANGLE = 82.0;      // Turn left to align with balls
    public static double INTAKE_FORWARD = 40.0;       // Forward while intaking
    public static double INTAKE_BACKWARD = 20.0;      // Backward after intake

    public static double PPG_EXTRA_FORWARD = 20.0;    // Extra forward before turning to shoot (PPG)
    public static double PGP_EXTRA_FORWARD = 8.0;     // Extra forward before turning to shoot (PGP)

    // Return to shooting position
    public static double SHOOT_HEADING = 0.0;         // Return to 0° for shooting

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

        // Start continuous PID update thread
        autonController.startPidUpdateThread();

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        autonController.stopPidUpdateThread();
        shooterController.shooterStop();
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

        // Initialize AutonController
        autonController = new AutonController(this, driveController, transferController,
                shooterController, intakeController, limelightController, autoShootController);

        telemetry.addLine("=== AUTON READY ===");
        telemetry.update();
    }

    private void executeAutonomousSequence() {
        // Start warmup in background
        Thread warmupThread = new Thread(() -> autonController.warmupShooter());
        warmupThread.start();

        // Move backward 50 inches
        autonController.moveRobot(-INITIAL_BACKWARD, MOVEMENT_SPEED);

        // Wait for warmup to complete (if not already)
        try {
            warmupThread.join(100); // Max 100ms wait
        } catch (InterruptedException e) {
            // Continue anyway
        }

        // Shoot preloaded balls
        autonController.quickShoot();

        // Turn and scan for pattern
        autonController.turnToHeading(PATTERN_SCAN_ANGLE);

        // Pattern detection
        int patternIndex = autonController.detectPattern();
        String patternName = (patternIndex == 0) ? "PPG" : (patternIndex == 1) ? "PGP" : "GPP";

        telemetry.addData("Detected Pattern", patternName);
        telemetry.update();

        // Move to appropriate line position based on pattern
        double positionDistance = PATTERN_POSITION_DISTANCE[patternIndex];
        autonController.moveRobot(positionDistance, MOVEMENT_SPEED);

        // Turn left 90 degrees to align with balls
        autonController.turnToHeading(getCurrentHeading() + LEFT_TURN_ANGLE);

        // Go forward while intaking
        intakeController.intakeFull();
        autonController.moveRobot(INTAKE_FORWARD, INTAKE_SPEED);

        // Brief intake time
        autonController.sleepWithPid(400);
        intakeController.intakeStop();

        // Go backward
        autonController.moveRobot(-INTAKE_BACKWARD, MOVEMENT_SPEED);

        // Pattern-specific repositioning
        if (patternIndex == 0) {  // PPG pattern
            autonController.turnToHeading(PATTERN_SCAN_ANGLE);
            // Go forward extra inches before turning to shoot
            autonController.moveRobot(PPG_EXTRA_FORWARD, MOVEMENT_SPEED);
        }

        if (patternIndex == 1) {  // PGP pattern
            autonController.turnToHeading(PATTERN_SCAN_ANGLE);
            // Go forward extra inches before turning to shoot
            autonController.moveRobot(PGP_EXTRA_FORWARD, MOVEMENT_SPEED);
        }

        // Turn to 0° heading for shooting
        autonController.turnToHeading(SHOOT_HEADING);

        // Shoot the balls we just intaked
        autonController.quickShoot();
    }

    private double getCurrentHeading() {
        driveController.updateOdometry();
        return Math.toDegrees(driveController.getHeading());
    }
}