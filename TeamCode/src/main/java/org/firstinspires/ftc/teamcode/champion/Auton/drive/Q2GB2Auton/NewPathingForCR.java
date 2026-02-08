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
@Autonomous(name = "New Pathing for CR", group = "Test")
public class NewPathingForCR extends LinearOpMode {
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
    // Distance parameters
    public static double INITIAL_BACKWARD = 35.0;
    public static double FIRST_BACKWARD_Y = -17.0;
    public static double SPLINE_Y = -45.0;
    public static double SPLINE_X = 35.0;
    public static double SECOND_SPLINE_X = 50.0;
    public static double SECOND_SPLINE_Y = -52.0;
    public static double THIRD_SPLINE_X = 35.0;
    public static double THIRD_SPLINE_Y = -15.0;
    public static double ENDING_DISTANCE = 30.0;
    public static double FOURTH_SPLINE_X = 70.0;
    public static double FOURTH_SPLINE_Y = -60.0;
    public static double FIFTH_SPLINE_X = 35.0;
    public static double FIFTH_SPLINE_Y = -10.0;

    // turning angle parameters
    public static double SPLINE_ANGLE = -50.0;
    public static double SECOND_SPLINE_ANGLE = -110.0;
    public static double THIRD_SPLINE_ANGLE = 100.0;
    public static double FOURTH_SPLINE_ANGLE = -60.0;
    public static double FIFTH_SPLINE_ANGLE = 100.0;

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
                .lineToX(INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(Initial_Forward);

        // SHOOT
        autoMethod.shootBalls();

        // SPLINE FOR INTAKE (FIRST LINE)
        autoMethod.intakeSpline(SPLINE_X, -SPLINE_Y, -SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (FIRST LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward = tankDrive.actionBuilder(currentPose)
                .lineToY(-FIRST_BACKWARD_Y)
                .build();
        Actions.runBlocking(Backward);

        // AUTO AIM AND SHOOT (FIRST LINE)
        autoMethod.shootBalls();

        // SPLINE FOR INTAKE (SECOND LINE)
        autoMethod.intakeSpline(SECOND_SPLINE_X, -SECOND_SPLINE_Y, -SECOND_SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (SECOND LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action splineBackward = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(THIRD_SPLINE_X, -THIRD_SPLINE_Y), Math.toRadians(-THIRD_SPLINE_ANGLE))
                .build();
        Actions.runBlocking(splineBackward);

        // AUTO AIM AND SHOOT (SECOND LINE)
        autoMethod.shootBalls();

        // SPLINE FOR INTAKE (THIRD LINE)
        autoMethod.intakeSpline(FOURTH_SPLINE_X, -FOURTH_SPLINE_Y, -FOURTH_SPLINE_ANGLE);

        // GO BACK FOR SHOOTING (THIRD LINE)
        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action splineBackward2 = tankDrive.actionBuilder(currentPose)
                .setReversed(true)
                .splineTo(new Vector2d(FIFTH_SPLINE_X, -FIFTH_SPLINE_Y), Math.toRadians(-FIFTH_SPLINE_ANGLE))
                .build();
        Actions.runBlocking(splineBackward2);

        // AUTO AIM AND SHOOT (SECOND LINE)
        autoMethod.shootBalls();

    }
}