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
import org.firstinspires.ftc.teamcode.champion.RobotState;
import org.firstinspires.ftc.teamcode.champion.controller.AutonTurretController;
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
@Autonomous(name = "CloseBlue GB2", group = "Competition")
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

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;
    public static double AUTON_START_X = 49.6;
    public static double AUTON_START_Y = 9.0;
    public static double AUTON_START_HEADING = 0.0;

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 3100.0;
    public static double CONSTANT_RAMP_ANGLE = 0.449;

    // Distance parameters
    public static double INITIAL_BACKWARD = -40.0;
    public static double INTAKE_FORWARD = 30.0;
    public static double INTAKE_BACKWARD = 30.0;
    public static double INTAKE_SECOND_BACKWARD = 20.0;

    public static double SECOND_BACKWARD = 22.0;
    public static double SECOND_FORWARD= 24.0;
    public static double ENDING_DISTANCE = 30.0;

    // turning angle parameters
    public static double DEGREE_ZERO = 0.0;
    public static double PICK_UP_ANGLE = 90.0;

    // turning perfection
    public static double HEADING_CORRECTION_KP = 0.015;
    public static double HEADING_CORRECTION_MAX_VEL = 0.3;
    public static int HEADING_STABLE_SAMPLES = 3;
    public static double HEADING_TIMEOUT_MS = 280;

    // Timing parameters
    public static long INTAKE_TIME_MS = 280;
    public static long SHOOT_TIME_MS = 3000;
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    // turret angles
    public static double AUTO_AIM_LEFT = -40.0;

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

        sleep(100);

        // Execute autonomous sequence using RoadRunner
        executeAutonomousSequence();

        // Cleanup
        cleanup();
    }

    private void initializeRobot() {
        driveController = new SixWheelDriveController(this);
        driveController.resetOdometry();  // <-- ADDED THIS LINE
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
        shooterController = new NewShooterController(shooterMotor1,shooterMotor2);

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

    /* 1. FINISH TUNING FOR TURNING
       2. FIND THE RAMP ANGLE AND SHOOTING RPM FOR BULLET SHOT
       3. IMPLEMENT THE TURRET IN MY CODE TO REDUCE THE TURNING MOVEMENT
       4. INCREASE THE SPEED AND SAVE TIME AS SOON AS POSSIBLE
    */
    private void executeAutonomousSequence() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        // 1. Go backward
        Action moveBackward1 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward1);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 3. shoot 3 balls
        autoAimTurretLeft();
        shootBalls();

        // 4. turn to pickup angle (90)
        Action turnLeft2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turnLeft2);
        HeadingCorrection(PICK_UP_ANGLE, 0.5);

        // 5. Go forward while intake (first line)
        intakeForwardRoadRunner();
        currentPose = tankDrive.pinpointLocalizer.getPose();
        turretField.disable();

        // 6. Go backward after intake (first line)
        Action moveBackward2 = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y - INTAKE_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward2);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 8. Shoot balls
        // no autoaim because of the intakeForward
        shootBalls();

        // 9. turn to 0 degree for going backward
        Action turnRight2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(DEGREE_ZERO))
                .build();
        Actions.runBlocking(turnRight2);
        HeadingCorrection(DEGREE_ZERO, 0.5);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 10. go backward
        Action moveBackward3 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - SECOND_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward3);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 11. turn left 90 degree for pickup (second line)
        Action turnLeft3 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turnLeft3);
        HeadingCorrection(PICK_UP_ANGLE, 0.5);

        // 12. forward intake for pickup (second line)
        intakeForwardRoadRunner();
        currentPose = tankDrive.pinpointLocalizer.getPose();
        turretField.disable();

        // 13. backward intake (second line)
        Action moveBackward4 = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y - INTAKE_SECOND_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward4);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 14. facing zero degree
        Action turnRight = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(DEGREE_ZERO))
                .build();
        Actions.runBlocking(turnRight);
        HeadingCorrection(DEGREE_ZERO, 0.5);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 15. forward with turret as it changes its heading, it would have to be with forward, not intake
        forwardTurret();
        turretField.disable();

        // 17. shoot balls
        shootBalls();

        // 19. Ending pose
        Action moveForward5 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x - ENDING_DISTANCE)
                .build();
        Actions.runBlocking(moveForward5);

    }

    private void shootBalls() {
        // Wait for RPM stabilization
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 200) {
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
        int ballsShotCount = 0;
        boolean lastBallState = false; // sensor reading from previous state - for counting exactly one for one ball

        while (opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            // Get current values
            boolean ballDetected = isBallAtUptake();

            // Count balls shot
            if (ballDetected && !lastBallState) {
                ballsShotCount++;
                if (ballsShotCount >= 3) {
                    sleep(200);  // Let last ball clear
                    break;
                }
            }
            lastBallState = ballDetected;

            // Update all controllers to keep them running
            intakeController.update();
            transferController.update();
            uptakeController.update();

            sleep(30);
        }

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        sleep(200);
    }

    private void forwardTurret(){
        if(turretField != null){
            turretField.setTargetFieldAngle(AUTO_AIM_LEFT);
            turretField.enable();
        }

        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + SECOND_FORWARD)
                .build();

        Action intakeAction = new Action() {
            private Action moveAction = moveForward;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                if (turretField != null && turretField.isEnabled()){
                    turret.update();
                    turretField.update(
                            Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble())
                    );
                }
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);
    }

    private void intakeForwardRoadRunner() {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        if (turretField != null){
            turretField.setTargetFieldAngle(AUTO_AIM_LEFT);
            turretField.enable(); // starts the controller non blocking; before, the autoaim was blocking
        }

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y + INTAKE_FORWARD)
                .build();

        // Create a custom action that combines RoadRunner movement with intake control
        Action intakeAction = new Action() {
            private Action moveAction = moveForward;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                checkUptakeSwitch();
                intakeController.update();
                transferController.update();
                uptakeController.update();
                if (turretField != null && turretField.isEnabled()){
                    turret.update();
                    turretField.update(
                            Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble())
                    );
                }
                return moveAction.run(packet);
            }
        };

        // Run the combined action
        Actions.runBlocking(intakeAction);

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
            uptakeController.update();
            if (turretField != null && turretField.isEnabled()){
                turret.update();
                turretField.update(
                        Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble())
                );
            }
            sleep(30);
        }

        // Stop all systems
        intakeModeActive = false;
        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();
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
        Pose2d rawPose = tankDrive.pinpointLocalizer.getPose();
        double rawX = rawPose.position.x;
        double rawY = rawPose.position.y;

        // === COORDINATE TRANSFORMATION ===
        // SWAP_XY = true, NEGATE_X = false, NEGATE_Y = true
        double deltaX = rawY;       // Swapped, not negated
        double deltaY = -rawX;      // Swapped, then negated

        // Add delta to starting position
        double fieldX = AUTON_START_X + deltaX;
        double fieldY = AUTON_START_Y + deltaY;

        // Save CORRECTED field coordinates
        PoseStorage.currentPose = new Pose2d(
                new Vector2d(fieldX, fieldY),
                rawPose.heading
        );
        if (turret != null) {
            PoseStorage.turretAngle = turret.getTurretAngle();
        }
        telemetry.addLine("=== AUTON COMPLETE ===");
        telemetry.addData("Raw Odom", "x=%.1f, y=%.1f", rawX, rawY);
        telemetry.addData("Delta (corrected)", "dx=%.1f, dy=%.1f", deltaX, deltaY);
        telemetry.addData("Field Pose SAVED", "x=%.1f, y=%.1f, h=%.1f°",
                fieldX, fieldY, rawPose.heading);
        telemetry.addData("Turret SAVED", "%.1f°", PoseStorage.turretAngle);
        telemetry.update();
        sleep(2000);

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

    private void HeadingCorrection(double targetAngleDegrees, double toleranceDegrees) {
        final double kP = HEADING_CORRECTION_KP;
        final double maxAngularVel = HEADING_CORRECTION_MAX_VEL;
        final int stableSamplesRequired = HEADING_STABLE_SAMPLES;
        final double timeoutMs = HEADING_TIMEOUT_MS;

        final int maxAttempts = 15;
        int attemptCount = 0;
        int stableCount = 0;

        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opModeIsActive()
                && attemptCount < maxAttempts
                && timeout.milliseconds() < timeoutMs) {

            tankDrive.updatePoseEstimate();
            Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
            double currentAngleDeg = Math.toDegrees(currentPose.heading.toDouble());

            // normalize error
            double headingError = targetAngleDegrees - currentAngleDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            if (Math.abs(headingError) <= toleranceDegrees) {
                stableCount++;
                if (stableCount >= stableSamplesRequired) break;
            } else {
                stableCount = 0;
            }

            double angularVel = kP * headingError;
            angularVel = Math.max(-maxAngularVel, Math.min(maxAngularVel, angularVel));

            tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), angularVel));

            sleep(30);
            attemptCount++;
        }

        tankDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(50);
    }

    private void autoAimTurretLeft () {
        if (turretField == null) return;

        turretField.autoAim(
                AUTO_AIM_LEFT,
                () -> Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble()),
                () -> opModeIsActive()
        );
    }


}