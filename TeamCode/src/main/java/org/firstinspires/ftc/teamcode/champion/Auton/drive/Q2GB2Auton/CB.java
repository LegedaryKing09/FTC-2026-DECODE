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
@Autonomous(name = "CLOSE BLUE - 9 BALLS", group = "Competition")
public class CB extends LinearOpMode {
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
    public static double CONSTANT_SHOOTER_RPM = 3200.0;
    public static double CONSTANT_RAMP_ANGLE = 0.0;
    // Distance
    public static double INITIAL_BACKWARD = 30.0;
    public static double INTAKE_DISTANCE = 25.0;
    public static double INTAKE_BACKWARD = 25.0;
    public static double SECOND_LINE_PICKUP_BACKWARD = 25.0;

    // turning angle
    public static double PICK_UP_ANGLE = 45.0;
    public static double ZERO_DEGREE = -45.0;
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
                .lineToX(currentPose.position.x + INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(Initial_Forward);

        // SHOOT
        autoMethod.shootBalls();

        // TURN TO 90 DEGREE ANGLE
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Turn1 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(Turn1);
        autoMethod.HeadingCorrection(PICK_UP_ANGLE, 0.5);

        // GO FORWARD WITH INTAKE (FIRST LINE)
        autoMethod.intakeSForward(INTAKE_DISTANCE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward1 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - INTAKE_BACKWARD)
                .build();
        Actions.runBlocking(Backward1);

        // SHOOT (FIRST LINE)
        autoMethod.shootBalls();

        // TURN TO 0 DEGREE ANGLE  (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Turn2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(ZERO_DEGREE))
                .build();
        Actions.runBlocking(Turn2);
        autoMethod.HeadingCorrection(ZERO_DEGREE, 0.5);

        // GO BACK FOR PICKUP (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward2 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - SECOND_LINE_PICKUP_BACKWARD)
                .build();
        Actions.runBlocking(Backward2);

        // TURN TO 90 DEGREE ANGLE  (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Turn3 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(Turn3);
        autoMethod.HeadingCorrection(PICK_UP_ANGLE, 0.5);

        // GO FORWARD WITH INTAKE (SECOND LINE)
        autoMethod.intakeSForward(INTAKE_DISTANCE);

        // GO BACK AFTER INTAKE (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward3 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - INTAKE_BACKWARD)
                .build();
        Actions.runBlocking(Backward3);

        // TURN TO 0 DEGREE ANGLE  (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Turn4 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(ZERO_DEGREE))
                .build();
        Actions.runBlocking(Turn4);
        autoMethod.HeadingCorrection(ZERO_DEGREE, 0.5);

        // GO FORWARD FOR SHOOTING (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Forward = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + SECOND_LINE_PICKUP_BACKWARD)
                .build();
        Actions.runBlocking(Forward);

        // SHOOT (SECOND LINE)
        autoMethod.shootBalls();
    }
}