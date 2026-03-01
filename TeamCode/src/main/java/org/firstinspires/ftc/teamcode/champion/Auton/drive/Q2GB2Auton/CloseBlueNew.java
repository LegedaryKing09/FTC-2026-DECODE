package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutonController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
@Autonomous(name = "CB - 12 BALLS", group = "Test")
public class CloseBlueNew extends LinearOpMode {
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
    public static double INITIAL_BACKWARD = 30.0;
    public static double SPLINE_X = 32.0;
    public static double SPLINE_Y = -38.0;
    public static double SECOND_SPLINE_X = 40.0;
    public static double SECOND_SPLINE_Y = -53.0;
    public static double THIRD_SPLINE_X = 48.0;
    public static double THIRD_SPLINE_Y = -63.0;
    public static double GOBACK_SPLINE_X = 38.0;
    public static double GOBACK_SPLINE_Y = -5.0;

    // turning angle parameters
    public static double SPLINE_ANGLE = -135.0;
    public static double SECOND_SPLINE_ANGLE = -135.0;
    public static double THIRD_SPLINE_ANGLE = -135.0;
    public static double GOBACK_ANGLE = 90.0;
    public static double TURNING_ANGLE = -90.0;

    // ===========================
    private final ElapsedTime globalTimer = new ElapsedTime();
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    @Override
    public void runOpMode() {
        initializeRobot();

        // Set yaw scalar and re-apply to pinpoint (constructor used default of -1.0)
        driveController.YAW_SCALAR = 1;
        driveController.getPinpoint().setYawScalar(driveController.YAW_SCALAR);

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        try {
            autoMethod = new AutonMethods(
                    this,
                    driveController,
                    transferController,
                    uptakeController,
                    shooterController,
                    intakeController,
                    rampController,
                    autonController,
                    tankDrive,
                    turret
            );
            autoMethod.uptakeSwitch = uptakeSwitch;
            autoMethod.telemetry = telemetry;
            AutonMethods.AUTON_START_X = 18.5;
            AutonMethods.AUTON_START_Y = 16;
            AutonMethods.AUTON_START_HEADING = -135;
            AutonMethods.SHOOT_TARGET_X = 10;
            AutonMethods.SHOOT_TARGET_Y = 10;
            AutonMethods.useHeadingOnlyAim = false;
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

        // Enable turret auto-aim
        autoMethod.autoAimTurretLeft();

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

        // Initialize auton_controller
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

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turn = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(TURNING_ANGLE))
                .build();
        Actions.runBlocking(turn);

        autoMethod.aimAndShoot();

        // SPLINE FOR INTAKE (FIRST LINE)
        autoMethod.intakeSpline(SPLINE_X, SPLINE_Y, SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(GOBACK_SPLINE_X,GOBACK_SPLINE_Y), Math.toRadians(GOBACK_ANGLE))
                .build();
        Actions.runBlocking(Backward);

        autoMethod.aimAndShoot();

        // SPLINE FOR INTAKE (SECOND LINE)
        autoMethod.intakeSpline(SECOND_SPLINE_X, SECOND_SPLINE_Y, SECOND_SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward2 = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(GOBACK_SPLINE_X,GOBACK_SPLINE_Y), Math.toRadians(GOBACK_ANGLE))
                .build();
        Actions.runBlocking(Backward2);

        // AUTO AIM AND SHOOT (SECOND LINE)
//        autoMethod.autoAimTurretLeft();
        autoMethod.aimAndShoot();

        // SPLINE FOR INTAKE (THIRD LINE)
        autoMethod.intakeSpline(THIRD_SPLINE_X, THIRD_SPLINE_Y, THIRD_SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (THIRD LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward3 = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(GOBACK_SPLINE_X+8,GOBACK_SPLINE_Y-5), Math.toRadians(GOBACK_ANGLE))
                .build();
        Actions.runBlocking(Backward3);

        // AUTO AIM AND SHOOT (SECOND LINE)
//        autoMethod.autoAimTurretLeft();
        autoMethod.aimAndShoot();

    }
}