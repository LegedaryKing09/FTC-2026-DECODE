package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.*;

@Config
@Autonomous(name = "Stable Basic Auton", group = "Competition")
public class BasicAuton extends LinearOpMode {

    // Controllers
    private SixWheelDriveController driveController;
    private TransferController transferController;
    private ShooterController shooterController;
    private IntakeController intakeController;
    private LimelightAlignmentController limelightController;
    private AutoShootController autoShootController;
    private RampController rampController;
    private StableAutonController autonController;

    // Autonomous parameters (UNCHANGED)
    public static double CONSTANT_SHOOTER_RPM = 2800.0;
    public static double CONSTANT_RAMP_ANGLE = 121.0;
    public static double MOVEMENT_SPEED = 0.65;
    public static double INTAKE_SPEED = 0.4;

    // Path parameters (UNCHANGED)
    public static double INITIAL_BACKWARD = 38.0;
    public static double PATTERN_SCAN_ANGLE = -45.0;
    public static double[] PATTERN_POSITION_DISTANCE = {-35, -20, 0};
    public static double LEFT_TURN_ANGLE = 71.0;
    public static double INTAKE_FORWARD = 34.0;
    public static double INTAKE_BACKWARD = 17.0;
    public static double PPG_EXTRA_FORWARD = 20.0;
    public static double PGP_EXTRA_FORWARD = 8.0;
    public static double SHOOT_HEADING = 0.0;

    private final ElapsedTime globalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize
        initializeRobot();

        // Simple ready message
        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Start shooter at constant RPM
        shooterController.setShooterRPM(CONSTANT_SHOOTER_RPM);

        // Start single PID thread
        autonController.startPidUpdateThread();

        // Execute sequence (UNCHANGED LOGIC)
        executeAutonomousSequence();

        // Cleanup
        cleanup();

        // Final telemetry
        telemetry.addData("Complete", "%.1fs", globalTimer.seconds());
        telemetry.update();
    }

    private void initializeRobot() {
        // Initialize base controllers
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);

        // Set initial values
        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        // Try to initialize vision (optional)
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

        // Use stable controller
        autonController = new StableAutonController(
                this, driveController, transferController, shooterController,
                intakeController, limelightController, autoShootController
        );
    }

    private void executeAutonomousSequence() {
        // PHASE 1: Move backward (NO separate warmup thread)
        autonController.moveRobot(-INITIAL_BACKWARD, MOVEMENT_SPEED);

        // Quick warmup check (inline, no thread)
        autonController.warmupShooter();

        // Shoot preloaded
        autonController.quickShoot();

        // PHASE 2: Turn and detect pattern
        autonController.turnToHeading(PATTERN_SCAN_ANGLE);

        int patternIndex = autonController.detectPattern();

        // Move to position
        double positionDistance = PATTERN_POSITION_DISTANCE[patternIndex];
        autonController.moveRobot(positionDistance, MOVEMENT_SPEED);

        // Turn to face balls
        autonController.turnToHeading(PATTERN_SCAN_ANGLE+LEFT_TURN_ANGLE);

        // PHASE 3: Intake sequence (direct control, no thread)
        intakeController.intakeFull();
        autonController.moveRobot(INTAKE_FORWARD, INTAKE_SPEED);

        // Brief intake time
        sleep(400);
        intakeController.intakeStop();

        // Move backward
        autonController.moveRobot(-INTAKE_BACKWARD, MOVEMENT_SPEED);

        // PHASE 4: Position for final shot
        if (patternIndex == 0) {  // PPG
            autonController.turnToHeading(PATTERN_SCAN_ANGLE);
            autonController.moveRobot(PPG_EXTRA_FORWARD, MOVEMENT_SPEED);
        } else if (patternIndex == 1) {  // PGP
            autonController.turnToHeading(PATTERN_SCAN_ANGLE);
            autonController.moveRobot(PGP_EXTRA_FORWARD, MOVEMENT_SPEED);
        }

        // Turn to shoot
        autonController.turnToHeading(SHOOT_HEADING);

        // Final shot
        autonController.quickShoot();
    }

    private void cleanup() {
        // Stop PID thread
        autonController.stopPidUpdateThread();

        // Stop all motors
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();
    }
}