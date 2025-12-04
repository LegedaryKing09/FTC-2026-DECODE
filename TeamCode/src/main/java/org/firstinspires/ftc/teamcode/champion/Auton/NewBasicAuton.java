package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutonController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@Autonomous(name = "Basic Auton NEW ROBOT", group = "Competition")
public class NewBasicAuton extends LinearOpMode {
    // Controllers
    SixWheelDriveController driveController;
    NewTransferController transferController;
    UptakeController uptakeController;
    NewShooterController shooterController;
    NewIntakeController intakeController;
    LimelightAlignmentController limelightController;
    NewAutoShootController autoShootController;
    NewRampController rampController;
    NewAutonController autonController;

    // Shooter and ramp settings
    public static double CONSTANT_SHOOTER_RPM = 2700.0;
    public static double CONSTANT_RAMP_ANGLE = 119.0;
    public static double MOVEMENT_SPEED = 0.6;
    public static double INTAKE_SPEED = 0.25;

    // ============ PATH PARAMETERS ============
    public static double INITIAL_BACKWARD = 40.0;
    public static double PATTERN_SCAN_ANGLE = -67.0;
    public static double ALIGNMENT_ANGLE = -45.0;
    public static double[] PATTERN_POSITION_DISTANCE = {
            -58.0,  // PPG
            -34.0,  // PGP
            -8.0    // GPP
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
        shooterController.setTargetRPM(CONSTANT_SHOOTER_RPM);
        shooterController.startShooting();

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

        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
        telemetry.addData("✓ Drive", "OK");

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            telemetry.addData("✓ Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Intake", "NOT FOUND");
        }
        intakeController = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            telemetry.addData("✓ Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Transfer", "NOT FOUND");
        }
        transferController = new NewTransferController(transferMotor);

        // Initialize uptake (NEW!)
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            telemetry.addData("✓ Uptake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Uptake", "NOT FOUND");
        }
        uptakeController = new UptakeController(uptakeServo);

        // Initialize shooter
        DcMotor shooterMotor = null;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            telemetry.addData("✓ Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter", "NOT FOUND");
        }
        shooterController = new NewShooterController(shooterMotor);

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
            telemetry.addData("✓ Ramp", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Ramp", "NOT FOUND");
            rampController = null;
        }

        // Try to initialize Limelight (teammate will fix issues)
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new NewAutoShootController(this, driveController, shooterController,
                    intakeController, transferController, uptakeController, limelightController, rampController);
            telemetry.addData("✓ Vision", "OK (teammate fixing)");
        } catch (Exception e) {
            telemetry.addData("⚠ Vision", "unavailable (teammate fixing)");
            limelightController = null;
            autoShootController = null;
        }

        // Initialize AutonController with NEW controllers
        autonController = new NewAutonController(
                this,                   // LinearOpMode opMode
                driveController,        // SixWheelDriveController driveController
                transferController,     // NewTransferController transferController
                uptakeController,       // UptakeController uptakeController
                shooterController,      // NewShooterController shooterController
                intakeController,       // NewIntakeController intakeController
                limelightController,    // LimelightAlignmentController limelightController
                autoShootController,    // NewAutoShootController autoShootController
                rampController          // NewRampController rampController
        );

        telemetry.update();
        sleep(1000);
    }

    private void executeAutonomousSequence() {
        // Warm up the shooter
        autonController.warmupShooter();

        // Go back 40 inches
        autonController.moveRobot(-INITIAL_BACKWARD, MOVEMENT_SPEED);

        // Shoot preloaded balls (with REAL-TIME RPM compensation)
        autonController.quickShoot();

        // Turn and scan for pattern
        autonController.turnToHeading(PATTERN_SCAN_ANGLE);

        // Pattern detection
        int patternIndex = autonController.detectPattern();
        telemetry.addData("Pattern Detected", patternIndex);
        telemetry.update();

        // Turn to alignment angle (90 degrees to the balls)
        autonController.turnToHeading(ALIGNMENT_ANGLE);

        // Move to appropriate line position based on pattern
        double positionDistance = PATTERN_POSITION_DISTANCE[patternIndex];
        autonController.moveRobot(positionDistance, MOVEMENT_SPEED);

        // Turn left 90 degrees to intake
        autonController.turnToHeading(ALIGNMENT_ANGLE + LEFT_TURN_ANGLE);

        // Go forward while intaking
        intakeController.setState(true);
        intakeController.update();
        autonController.moveRobot(INTAKE_FORWARD, INTAKE_SPEED);

        // Brief intake time
        sleep(400);
        intakeController.setState(false);
        intakeController.update();

        // Go backward
        autonController.moveRobot(-INTAKE_BACKWARD, MOVEMENT_SPEED);

        // Turn right 90 degrees
        autonController.turnToHeading(ALIGNMENT_ANGLE);

        // Move back to shooting position
        autonController.moveRobot(-positionDistance, MOVEMENT_SPEED);

        // Turn for shooting
        autonController.turnToHeading(SHOOT_HEADING);

        // Shoot the balls we just intaked (with REAL-TIME RPM compensation)
        autonController.quickShoot();
    }

    private void cleanup() {
        autonController.stopPidUpdateThread();
        shooterController.stopShooting();
        intakeController.setState(false);
        intakeController.update();
        transferController.stop();
        transferController.update();
        uptakeController.setState(false);
        uptakeController.update();
        driveController.stopDrive();
    }
}