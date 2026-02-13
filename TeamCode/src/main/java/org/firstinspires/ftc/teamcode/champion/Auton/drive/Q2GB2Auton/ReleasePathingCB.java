package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;
import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutonController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
@Autonomous(name = "RELEASE for CB - 6 balls", group = "Test")
public class ReleasePathingCB extends LinearOpMode {
    SixWheelDriveController driveController;
    NewTransferController transferController;
    UptakeController uptakeController;
    NewShooterController shooterController;
    NewIntakeController intakeController;
    NewRampController rampController;
    LimelightAlignmentController limelightController;
    NewAutoShootController autoShootController;
    NewAutonController autonController;
    AutoTankDrive tankDrive;
    TurretFieldController turretField;
    TurretController turret;
    AutonMethods autoMethod;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // ==================================

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 3400.0;
    public static double CONSTANT_RAMP_ANGLE = 0.0;
    // Distance parameters
    public static double INITIAL_BACKWARD = -30.0;
    public static double FIRST_BACKWARD_Y = -22.0;
    public static double SPLINE_Y = -52.0;
    public static double SPLINE_X = -11.0;
    public static double SECOND_SPLINE_X = 12.0;
    public static double SECOND_SPLINE_Y = -52.0;
    public static double THIRD_SPLINE_X = -5.0;
    public static double THIRD_SPLINE_Y = -54.0;
    public static double RELEASE_X = -20.0;
    public static double RELEASE_Y = -35.0;

    // turning angle parameters
    public static double RELEASE_ANGLE = -180.0;
    public static double SPLINE_ANGLE = -90.0;
    public static double SECOND_SPLINE_ANGLE = -180.0;
    public static double THIRD_SPLINE_ANGLE = -90.0;

    // ===========================
    private final ElapsedTime globalTimer = new ElapsedTime();
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    @Override
    public void runOpMode() {
        initializeRobot();

        // Define starting pose
        Pose2d startPose = new Pose2d(-55, -55, Math.toRadians(45));
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        try {
            autoMethod = new AutonMethods(
                    this,
                    driveController,
                    transferController,
                    uptakeController,
                    shooterController,
                    intakeController,
                    limelightController,
                    autoShootController,
                    rampController,
                    autonController,
                    tankDrive,
                    turretField,
                    turret
            );
            autoMethod.uptakeSwitch = uptakeSwitch;
        } catch (Exception e){
            //
        }

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Start shooter and keep it running
        shooterController.setTargetRPM(CONSTANT_SHOOTER_RPM);
        shooterController.startShooting();
        autoMethod.startShooterThread();

        sleep(100);

        // Execute autonomous sequence using RoadRunner
        executeAutonomousSequence();

        // Cleanup
        autoMethod.cleanup();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        driveController.resetOdometry();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            //
        }
        intakeController = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            //
        }
        transferController = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        CRServo uptakeServo2 = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "servo1");
            uptakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake: " + e.getMessage());
        }
        uptakeController = new UptakeController(uptakeServo, uptakeServo2);

        // Initialize uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            //
        }

        // Initialize shooter
        DcMotor shooterMotor1 = null;
        DcMotor shooterMotor2 = null;
        try {
            shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Shooter: " + e.getMessage());
        }
        shooterController = new NewShooterController(shooterMotor1, shooterMotor2);

        // initialize turret
        try {
            turret = new TurretController(this);
            turretField = new TurretFieldController(turret);
        } catch (Exception e) {
            //
        }

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
        } catch (Exception e) {
            //
        }

        // Initialize autoncontroller
        autonController = new NewAutonController(
                this,
                driveController,
                transferController,
                uptakeController,
                shooterController,
                intakeController,
                limelightController,
                autoShootController,
                rampController
        );

    }

    private void executeAutonomousSequence() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        // GO BACK FOR SHOOTING
        Action Initial_Forward = tankDrive.actionBuilder(currentPose)
                .lineToX(INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(Initial_Forward);

        // SHOOT
        autoMethod.autoAimTurretLeft();
        autoMethod.shootBalls();

        // SPLINE FOR INTAKE (FIRST LINE)
        autoMethod.intakeSpline(SPLINE_X, SPLINE_Y, SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(RELEASE_X, RELEASE_Y),Math.toRadians(RELEASE_ANGLE))
                .build();
        Actions.runBlocking(Backward);

        // AUTO AIM AND SHOOT (FIRST LINE)
        autoMethod.autoAimTurretLeft();
        autoMethod.shootBalls();

        // RELEASE
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action RELEASE = tankDrive.actionBuilder(currentPose)
                .splineTo(new Vector2d(0, -65),Math.toRadians(-90))
                .build();
        Actions.runBlocking(RELEASE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward2 = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(RELEASE_X, RELEASE_Y),Math.toRadians(RELEASE_ANGLE))
                .build();
        Actions.runBlocking(Backward2);



    }
}