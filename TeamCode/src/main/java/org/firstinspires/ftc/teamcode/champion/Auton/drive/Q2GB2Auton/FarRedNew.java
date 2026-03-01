package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2GB2Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "FAR RED 3", group = "Test")
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
        // Set turret to fixed position manually
        turret.setServoPosition(0.58);

        // Fire 3 balls (3s ramp-up between each shot handled inside shootBalls)
        autoMethod.shootBalls();

        // 10s pause
        sleep(10000);

        // Move forward 30in
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action forward = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + 30)
                .build();
        Actions.runBlocking(forward);
    }
}