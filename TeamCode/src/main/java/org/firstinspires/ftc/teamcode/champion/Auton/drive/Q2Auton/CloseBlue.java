package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.controller.AutonTurretController;
import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutonController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
@Autonomous(name = "Blue Close Auton Q2", group = "Competition")
public class CloseBlue extends LinearOpMode {
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
    AutonTurretController turret;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 3650.0;
    public static double CONSTANT_RAMP_ANGLE = -93.4;

    // Distance parameters (in INCHES)
    public static double INITIAL_BACKWARD = -50.0;
    public static double LEFT_TURN_ANGLE = 45.0;
    public static double PICK_UP_ANGLE = 90.0;
    public static double INTAKE_FORWARD = 30.0;
    public static double INTAKE_BACKWARD = 30.0;
    public static double SECOND_BACKWARD = 14.0;
    public static double ENDING_DISTANCE = 30.0;
    // Timing parameters
    public static long INTAKE_TIME_MS = 1000;
    public static long SHOOT_TIME_MS = 3600;
    public static double TURRET_TURN_ANGLE = 45.0;
    public static double SECOND_TURRET_TURN_ANGLE = 90.0;

    public boolean intakeModeActive = false;
    public boolean isShooting = false;
    public boolean uptakeStoppedBySwitch = false;
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;

    @Override
    public void runOpMode() {
        initializeRobot();


        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        tankDrive = new AutoTankDrive(hardwareMap, startPose);

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Start shooter and keep it running
        shooterController.setTargetRPM(CONSTANT_SHOOTER_RPM);
        shooterController.startShooting();
        startShooterThread();

        // Start continuous PID update thread for movement
        if (autonController != null) {
            autonController.startPidUpdateThread();
        }

        sleep(1000);

        // Execute autonomous sequence using RoadRunner
        executeAutonomousSequence();

        // Cleanup
        cleanup();
    }

    private void initializeRobot() {
        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
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
        DcMotor shooterMotorFirst = null;
        DcMotor shooterMotorSecond = null;
        try {
            shooterMotorFirst = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotorSecond = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            //
        }
        shooterController = new NewShooterController(shooterMotorFirst, shooterMotorSecond);

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
        } catch (Exception e) {
            //
        }

        // Initialize limelight
        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(NewAutoShootController.APRILTAG_ID);
            autoShootController = new NewAutoShootController(this, driveController, shooterController,
                    intakeController, transferController, uptakeController, limelightController, rampController);
        } catch (Exception e) {
            limelightController = null;
            autoShootController = null;
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

        telemetry.update();
        sleep(1000);

            try {
                turret = new AutonTurretController(this);
                telemetry.addData("Turret Initial Angle", "%.1f°", turret.getCurrentAngle());
            } catch (Exception e) {
                telemetry.addData("Turret Init Error", e.getMessage());
            }

    }

    private void executeAutonomousSequence() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        // 1. Go backward
        Action moveBackward1 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward1);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 2. shoot 3 balls
        shootBalls();
        sleep(50);

        // 3. turn left for first pickup
        Action turnLeft1 = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(LEFT_TURN_ANGLE))
                .build();
        Actions.runBlocking(turnLeft1);
        sleep(50);

        // 4. Go forward while intake (first)
        intakeForwardRoadRunner();
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 5. Go backward after intake (first)
        Action moveBackward2 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - INTAKE_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward2);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        Action TURNRIGHT1 = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-LEFT_TURN_ANGLE))
                .build();
        Actions.runBlocking(TURNRIGHT1);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);


        // 6. Shoot balls
        shootBalls();
        sleep(50);

        // 7. TURN RIGHT
        Action TURNRIGHT = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-LEFT_TURN_ANGLE))
                .build();
        Actions.runBlocking(TURNRIGHT);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 8. backward move for second pickup
        Action moveBackward3 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - SECOND_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward3);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 9. turn left for second pickup
        Action turnLeft3 = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turnLeft3);

        // 10. Go forward while intake (second)
        intakeForwardRoadRunner();
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 11. Go backward after intake (second)
        Action moveBackward4 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - INTAKE_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward4);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 12. turn right(second)
        Action turnRight = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turnRight);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 13. forward move (second)
        Action moveForward4 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + SECOND_BACKWARD)
                .build();
        Actions.runBlocking(moveForward4);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        //14. turn left for shooting
        Action turnRight2 = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(LEFT_TURN_ANGLE))
                .build();
        Actions.runBlocking(turnRight2);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);

        // 15. Shoot balls
        shootBalls();
        sleep(50);

        // 16. TURN RIGHT
        Action turnRight3 = tankDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-LEFT_TURN_ANGLE))
                .build();
        Actions.runBlocking(turnRight3);
        currentPose = tankDrive.pinpointLocalizer.getPose();
        sleep(50);


        // 15. ENDING POSE
        Action moveForward5 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - ENDING_DISTANCE)
                .build();
        Actions.runBlocking(moveForward5);

        sleep(50);
    }

    private void shootBalls() {
        // Wait for RPM stabilization
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 300) {
            if (Math.abs(shooterController.getRPM() - shooterController.getTargetRPM()) < 150) {
                break;
            }
            sleep(20);
        }

        // Start ALL systems for shooting
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        timer.reset();
//        int ballsShotCount = 0;
//        boolean lastBallState = false;

        while (opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            // Get current values for monitoring only
//            boolean ballDetected = isBallAtUptake();
//
//            // Count balls shot (detect when ball passes through)
//            if (ballDetected && !lastBallState) {
//                ballsShotCount++;
//            }
//            lastBallState = ballDetected;

            // Update all controllers to keep them running
            intakeController.update();
            transferController.update();
            uptakeController.update();

            sleep(50);
        }

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        sleep(1000);
    }

    private void turretAngleTurn(){

    }
    private void intakeForwardRoadRunner() {
        intakeModeActive = true;
//        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

//        uptakeController.setState(true);
//        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + INTAKE_FORWARD)
                .build();

        // Create a custom action that combines RoadRunner movement with intake control
        Action intakeAction = new Action() {
            private Action moveAction = moveForward;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                // Check uptake switch during movement
//                checkUptakeSwitch();

                // Update all controllers
                intakeController.update();
                transferController.update();
//                uptakeController.update();

                // Continue the movement action
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
//            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
//            uptakeController.update();
            sleep(50);
        }

        // Stop all systems
        intakeModeActive = false;
//        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

//        uptakeController.setState(false);
//        uptakeController.update();
    }

    private boolean isBallAtUptake() {
        if (uptakeSwitch == null) return true;
        return uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
    }

    private void startShooterThread() {
        runShooter = true;
        shooterThread = new Thread(() -> {
            while (runShooter && opModeIsActive()) {
                shooterController.update();
                if (rampController != null) {
                    rampController.update();
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        shooterThread.setPriority(Thread.MAX_PRIORITY);
        shooterThread.start();
    }

    private void cleanup() {
        if (autonController != null) {
            autonController.stopPidUpdateThread();
        }

        runShooter = false;
        if (shooterThread != null) {
            try {
                shooterThread.interrupt();
                shooterThread.join(500);
            } catch (Exception e) {
                // Thread cleanup failed - thread may have already stopped
            }
        }

        shooterController.stopShooting();
        intakeController.setState(false);
        intakeController.update();
        transferController.setState(false);
        transferController.update();
        uptakeController.setState(false);
        uptakeController.update();

        if (rampController != null) {
            rampController.stop();
        }

        telemetry.addLine("Cleanup complete");
        telemetry.addData("Total Time", "%.1f sec", globalTimer.seconds());
        telemetry.update();
    }

    private void checkUptakeSwitch() {
        if (uptakeSwitch == null || uptakeController == null || !intakeModeActive) {
            return;
        }

        boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

        if (ballDetected && !uptakeStoppedBySwitch) {
            uptakeController.setState(false);
            uptakeController.update();
            uptakeStoppedBySwitch = true;
        } else if (!ballDetected && uptakeStoppedBySwitch) {
            uptakeController.setState(true);
            uptakeController.update();
            uptakeStoppedBySwitch = false;
        }
    }
}