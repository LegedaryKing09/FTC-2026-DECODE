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
@Autonomous(name = "FR - 12 BALLS", group = "Test")
public class FarRedNew extends LinearOpMode {
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
    public static double CONSTANT_SHOOTER_RPM = 4300.0;
    public static double CONSTANT_RAMP_ANGLE = 0.34;
    // Distance parameters
    public static double COMEBACK_X = -8.0;
    public static double INITIAL_X = 5.0;
    public static double INITIAL_Y = 0.0;
    public static double SPLINE_Y = -43.0;
    public static double SPLINE_X = 32.0;
    public static double SECOND_SPLINE_X = 52.0;
    public static double SECOND_SPLINE_Y = -40.0;
    public static double THIRD_SPLINE_X = 72.0;
    public static double THIRD_SPLINE_Y = -40.0;
    public static double PICK_UP_DISTANCE_X = 0.0;
    public static double PICK_UP_DISTANCE_Y = -45.0;
    public static double LAST_SHOOTING_DISTANCE = -3.0;

    // turning angle parameters
    public static double INITIAL_ANGLE = 160.0;
    public static double SPLINE_ANGLE = -80.0;
    public static double SECOND_SPLINE_ANGLE = -80.0;
    public static double THIRD_SPLINE_ANGLE = -80.0;
    public static double PICKUP_ANGLE = -90.0;
    public static double RETURN_ANGLE = 0.0;
    public static double servoValue = 0.44;

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
            AutonMethods.AUTON_START_X = 96;
            AutonMethods.AUTON_START_Y = 137;
            AutonMethods.AUTON_START_HEADING = 0;
            AutonMethods.SHOOT_TARGET_X = 134;
            AutonMethods.SHOOT_TARGET_Y = 10;
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

        turret.setServoPosition(servoValue);
        autoMethod.shootBalls();
//        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
//        Action forward = tankDrive.actionBuilder(currentPose)
//                .lineToX(5)
//                .build();
//        Actions.runBlocking(forward);
//
//        // GO FOR PICKUP
//        autoMethod.intakeSpline(PICK_UP_DISTANCE_X, PICK_UP_DISTANCE_Y, PICKUP_ANGLE);
//
//        autoMethod.intakeYForward(COMEBACK_X);
//
//        autoMethod.aimAndPrepareShot();
//        autoMethod.shootBalls();
//
//        // GO BACK FOR SHOOTING (FIRST LINE)
//        currentPose = tankDrive.pinpointLocalizer.getPose();
//        Action RETURN = tankDrive.actionBuilder(currentPose)
//                .turnTo(Math.toRadians(RETURN_ANGLE))
//                .build();
//        Actions.runBlocking(RETURN);

        // SPLINE FOR INTAKE (FIRST LINE)
        autoMethod.intakeSpline(SPLINE_X, SPLINE_Y, SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(INITIAL_X,INITIAL_Y), Math.toRadians(INITIAL_ANGLE))
                .build();
        Actions.runBlocking(Backward);

        turret.setServoPosition(servoValue);
        autoMethod.shootBalls();

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action LEAVE = tankDrive.actionBuilder(currentPose)
                .lineToX(30)
                .build();
        Actions.runBlocking(LEAVE);
    }
}