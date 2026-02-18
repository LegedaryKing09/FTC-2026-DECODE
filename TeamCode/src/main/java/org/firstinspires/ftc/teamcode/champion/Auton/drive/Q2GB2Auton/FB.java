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
@Autonomous(name = "FAR BLUE - 9 BALLS", group = "Competition")
public class FB extends LinearOpMode {
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
    public static double CONSTANT_SHOOTER_RPM = 4100.0;
    public static double CONSTANT_RAMP_ANGLE = 0.6;
    // Distance
    public static double FIRST_PICKUP_DISTANCE = 30.0;
    public static double FIRST_LINE_BACKWARD = 30.0;
    public static double INTAKE_DISTANCE = 25.0;
    public static double INTAKE_BACKWARD = 25.0;
    public static double SECOND_LINE_PICKUP_BACKWARD = 25.0;

    // turning angle
    public static double PICK_UP_ANGLE = 90.0;
    public static double ZERO_DEGREE = 0.0;
    public static double SHOOTING_DEGREE = 0.0;

    // ===========================
    private final ElapsedTime globalTimer = new ElapsedTime();
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    @Override
    public void runOpMode() {
        initializeRobot();

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
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

        // SHOOT
        autoMethod.shootBalls();

        // GO FORWARD FOR PICKUP (FIRST LINE)
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward1 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + FIRST_PICKUP_DISTANCE)
                .build();
        Actions.runBlocking(Backward1);

        // TURN FOR PICKUP (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turn1 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turn1);
        autoMethod.HeadingCorrection(PICK_UP_ANGLE, 0.5);

        // INTAKE (FIRST LINE)
        autoMethod.intakeYForward(INTAKE_DISTANCE);

        // GO BACKWARD FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward2 = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y + FIRST_PICKUP_DISTANCE)
                .build();
        Actions.runBlocking(Backward2);

        // TURN FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action turn2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(ZERO_DEGREE))
                .build();
        Actions.runBlocking(turn2);
        autoMethod.HeadingCorrection(ZERO_DEGREE, 0.5);

        // GO BACK FOR SHOOTING
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward3 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - FIRST_LINE_BACKWARD)
                .build();
        Actions.runBlocking(Backward3);


    }
}