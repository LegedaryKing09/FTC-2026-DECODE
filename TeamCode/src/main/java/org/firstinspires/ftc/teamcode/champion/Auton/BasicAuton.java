package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.AutonController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

@Config
@Autonomous(name = "Basic Auton Running", group = "Competition")
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

    public static double CONSTANT_SHOOTER_RPM = 2700.0;
    public static double CONSTANT_RAMP_ANGLE = 119.0;
    public static double MOVEMENT_SPEED = 0.6;
    public static double INTAKE_SPEED = 0.25;

    // ============ PATH PARAMETERS ============
    public static double INITIAL_BACKWARD = 40.0;
    public static double PATTERN_SCAN_ANGLE = -67.0;
    public static double ALIGNMENT_ANGLE = -45.0;
    public static double[] PATTERN_POSITION_DISTANCE = {
            -58.0 ,  // PPG
            -34.0,  // PGP
            -8.0   // GPP
    };
    public static double LEFT_TURN_ANGLE = 90.0;
    public static double INTAKE_FORWARD = 22.0;
    public static double INTAKE_BACKWARD = 22.0;
    public static double SHOOT_HEADING = 0.0;

    private final ElapsedTime globalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
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
        cleanup();

        telemetry.addLine("=============================");
        telemetry.addLine("✓ AUTONOMOUS COMPLETE");
        telemetry.addLine("=============================");
        telemetry.addData("Total Time", "%.1fs", globalTimer.seconds());
        telemetry.update();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);

        // Set initial ramp angle
        rampController.setAngle(CONSTANT_RAMP_ANGLE);

        // Try to initialize Limelight
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new AutoShootController(this, driveController, shooterController,
                    intakeController, transferController, limelightController, rampController);
        } catch (Exception e) {
            telemetry.addLine("⚠ Vision system unavailable");
            telemetry.update();
        }

        // Initialize AutonController
        autonController = new AutonController(this, driveController, transferController,
                shooterController, intakeController, limelightController, autoShootController);

        // CRITICAL: Set the RampController so real-time RPM compensation can work
        autonController.setRampController(rampController);

    }



    private void executeAutonomousSequence() {
        //warm up the shooter
        autonController.warmupShooter();

        // go back 40 inches
        autonController.moveRobot(-INITIAL_BACKWARD, MOVEMENT_SPEED);

        // Shoot preloaded balls (with REAL-TIME RPM compensation)
        autonController.quickShoot();

        // Turn and scan for pattern
        autonController.turnToHeading(PATTERN_SCAN_ANGLE);

        // Pattern detection
        int patternIndex = autonController.detectPattern();

        // 90 degrees to the balls alignment
        autonController.turnToHeading(ALIGNMENT_ANGLE);

        // Move to appropriate line position based on pattern
        double positionDistance = PATTERN_POSITION_DISTANCE[patternIndex];
        autonController.moveRobot(positionDistance, MOVEMENT_SPEED);

        // Turn left 90 degrees to intake
        autonController.turnToHeading(ALIGNMENT_ANGLE + LEFT_TURN_ANGLE);

        // Go forward while intaking
        intakeController.intakeFull();
        autonController.moveRobot(INTAKE_FORWARD, INTAKE_SPEED);

        // Brief intake time
        sleep(400);
        intakeController.intakeStop();

        // Go backward
        autonController.moveRobot(-INTAKE_BACKWARD, MOVEMENT_SPEED);

        //turn right 90 degrees
        autonController.turnToHeading(ALIGNMENT_ANGLE);

        // move to shooting position
        autonController.moveRobot(-positionDistance, MOVEMENT_SPEED);

        //turn for shooting
        autonController.turnToHeading(SHOOT_HEADING);

        // Shoot the balls we just intaked (with REAL-TIME RPM compensation)
        autonController.quickShoot();
    }

    private void cleanup() {
        autonController.stopPidUpdateThread();
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();
    }
}