package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q2Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.controller.GB1AutoTankDrive;
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
import org.firstinspires.ftc.teamcode.champion.PoseStorage;

@Config
@Autonomous(name = "Blue Close Auton Q2 GB1", group = "Competition")
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
    GB1AutoTankDrive tankDrive;
    TurretFieldController turretField;
    TurretController turret;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 4000.0;
    public static double CONSTANT_RAMP_ANGLE = -130.0;

    // Distance parameters
    public static double INITIAL_BACKWARD = -40.0;
    public static double INTAKE_FORWARD = 36.0;
    public static double INTAKE_BACKWARD = 35.0;
    public static double INTAKE_SECOND_BACKWARD = 10.0;

    public static double SECOND_BACKWARD = 20.0;
    public static double ENDING_DISTANCE = 30.0;

    // turning angle parameters
    public static double DEGREE_ZERO = 0.0;
    public static double PICK_UP_ANGLE = 90.0;

    // turning perfection
    public static double HEADING_CORRECTION_KP = 0.015;
    public static double HEADING_CORRECTION_MAX_VEL = 0.3;
    public static int HEADING_STABLE_SAMPLES = 3;
    public static double HEADING_TIMEOUT_MS = 300;

    // Timing parameters
    public static long INTAKE_TIME_MS = 400;
    public static long SHOOT_TIME_MS = 3000;
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;
    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;

    // turret angles
    public static double AUTO_AIM_LEFT = 43.0;

    @Override
    public void runOpMode() {
        initializeRobot();

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        tankDrive = new GB1AutoTankDrive(hardwareMap, startPose);

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
            shooterMotorFirst = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotorSecond = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            //
        }
        shooterController = new NewShooterController(shooterMotorFirst);

        // initialize turret
        try {
            turret = new TurretController(this);
            turretField = new TurretFieldController(turret);
            telemetry.addData("Turret", "OK");
        } catch (Exception e) {
            telemetry.addData("Turret", "FAILED: " + e.getMessage());
        }

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
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

        telemetry.update();
    }

    // ========== DEBUG HELPER ==========
    private void debugTurret(String phase) {
        if (turret == null || turretField == null) return;

        double robotHeading = getHeadingDegrees();
        double turretAngle = turret.getTurretAngle();
        double fieldTarget = turretField.getTargetFieldAngle();
        double fieldError = turretField.getFieldError();

        // Calculate what field angle turret is actually facing
        double actualFieldFacing = robotHeading + turretAngle;
        while (actualFieldFacing > 180) actualFieldFacing -= 360;
        while (actualFieldFacing < -180) actualFieldFacing += 360;

        // Calculate required turret angle
        double requiredTurret = fieldTarget - robotHeading;

        telemetry.addLine("=== " + phase + " ===");
        telemetry.addData("Robot Heading", "%.1f°", robotHeading);
        telemetry.addData("Turret Angle", "%.1f°", turretAngle);
        telemetry.addData("Field Target", "%.1f°", fieldTarget);
        telemetry.addData("Actual Field Facing", "%.1f°", actualFieldFacing);
        telemetry.addData("Required Turret", "%.1f°", requiredTurret);
        telemetry.addData("Field Error", "%.1f°", fieldError);
        telemetry.addData("Turret Enabled", turretField.isEnabled());
        telemetry.addData("In Dead Zone", turretField.isInDeadZone());
        telemetry.update();
    }

    private double getHeadingDegrees() {
        tankDrive.updatePoseEstimate();
        return Math.toDegrees(tankDrive.pinpointLocalizer.getPose().heading.toDouble());
    }

    private void updateTurret() {
        if (turret == null || turretField == null || !turretField.isEnabled()) return;
        turret.update();
        turretField.update(getHeadingDegrees());
    }

    private void executeAutonomousSequence() {
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();

        // 1. Go backward
        Action moveBackward1 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(moveBackward1);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 3. shoot 3 balls
        debugTurret("BEFORE 1ST AUTO-AIM");
        autoAimTurretLeft();
        debugTurret("AFTER 1ST AUTO-AIM");
        sleep(500); // Pause to see telemetry

        shootBalls();

        // Disable turret before turn
        turretField.disable();
        debugTurret("AFTER DISABLE (before turn)");
        sleep(500);

        // 4. turn to pickup angle (90)
        Action turnLeft2 = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(PICK_UP_ANGLE))
                .build();
        Actions.runBlocking(turnLeft2);
        HeadingCorrection(PICK_UP_ANGLE, 0.5);

        debugTurret("AFTER TURN TO 90°");
        sleep(500);

        // 5. Go forward while intake (first line) - turret tracks
        debugTurret("BEFORE INTAKE FORWARD");
        intakeForwardRoadRunner();
        debugTurret("AFTER INTAKE FORWARD");
        sleep(500);

        currentPose = tankDrive.pinpointLocalizer.getPose();

        // ========== THIS IS WHERE THE PROBLEM HAPPENS ==========
        // 6. Go backward after intake - turret should KEEP tracking!
        telemetry.addLine("=== STARTING BACKWARD MOVE ===");
        telemetry.addData("Turret Enabled BEFORE", turretField.isEnabled());
        telemetry.update();
        sleep(300);

        Action moveBackward2 = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y - INTAKE_BACKWARD)
                .build();

        // Run with turret tracking!
        Action backwardWithTurret = new Action() {
            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                // UPDATE TURRET DURING BACKWARD MOVEMENT!
                updateTurret();

                // Debug in dashboard
                if (turretField != null) {
                    packet.put("Debug/RobotHeading", getHeadingDegrees());
                    packet.put("Debug/TurretAngle", turret.getTurretAngle());
                    packet.put("Debug/FieldError", turretField.getFieldError());
                    packet.put("Debug/TurretEnabled", turretField.isEnabled() ? 1 : 0);
                    packet.put("Debug/DeadZone", turretField.isInDeadZone() ? 1 : 0);
                }

                return moveBackward2.run(packet);
            }
        };
        Actions.runBlocking(backwardWithTurret);

        debugTurret("AFTER BACKWARD (before shoot)");
        sleep(500);

        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 8. Shoot balls
        shootBalls();

        debugTurret("AFTER 2ND SHOOT");

        turretField.disable();

        // Rest of sequence...
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

        // 13. backward intake (second line)
        Action moveBackward4 = tankDrive.actionBuilder(currentPose)
                .lineToY(currentPose.position.y - INTAKE_BACKWARD)
                .build();
        // Use turret tracking version
        Action backward4WithTurret = new Action() {
            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                updateTurret();
                return moveBackward4.run(packet);
            }
        };
        Actions.runBlocking(backward4WithTurret);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 14. facing zero degree
        Action turnRight = tankDrive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(DEGREE_ZERO))
                .build();
        Actions.runBlocking(turnRight);
        HeadingCorrection(DEGREE_ZERO, 0.5);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 15. forward move
        Action moveForward4 = tankDrive.actionBuilder(currentPose)
                .lineToX(currentPose.position.x + SECOND_BACKWARD)
                .build();
        Actions.runBlocking(moveForward4);
        currentPose = tankDrive.pinpointLocalizer.getPose();

        // 17. shoot balls
        shootBalls();

        turretField.disable();

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
            updateTurret(); // Keep tracking while waiting
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
        boolean lastBallState = false;

        while (opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            boolean ballDetected = isBallAtUptake();

            if (ballDetected && !lastBallState) {
                ballsShotCount++;
                if (ballsShotCount >= 3) {
                    sleep(200);
                    break;
                }
            }
            lastBallState = ballDetected;

            intakeController.update();
            transferController.update();
            uptakeController.update();

            // Keep turret tracking during shooting
            updateTurret();

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

    private void intakeForwardRoadRunner() {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Enable turret tracking
        if (turretField != null){
            turretField.setTargetFieldAngle(AUTO_AIM_LEFT);
            turretField.enable();
            debugTurret("INTAKE - just enabled");
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

        // Combined action
        Action intakeAction = new Action() {
            private Action moveAction = moveForward;
            private int updateCount = 0;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                checkUptakeSwitch();
                intakeController.update();
                transferController.update();
                uptakeController.update();

                // Turret tracking
                if (turretField != null && turretField.isEnabled()){
                    turret.update();
                    double heading = getHeadingDegrees();
                    turretField.update(heading);

                    // Debug every 10 updates
                    updateCount++;
                    if (updateCount % 10 == 0) {
                        packet.put("Intake/Heading", heading);
                        packet.put("Intake/TurretAngle", turret.getTurretAngle());
                        packet.put("Intake/FieldError", turretField.getFieldError());
                        packet.put("Intake/DeadZone", turretField.isInDeadZone() ? 1 : 0);
                    }
                }
                return moveAction.run(packet);
            }
        };

        Actions.runBlocking(intakeAction);

        // Keep intake running after stopping
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            checkUptakeSwitch();
            intakeController.update();
            transferController.update();
            uptakeController.update();

            // Keep turret tracking
            if (turretField != null && turretField.isEnabled()){
                turret.update();
                turretField.update(getHeadingDegrees());
            }
            sleep(30);
        }

        // Stop intake systems (turret stays enabled!)
        intakeModeActive = false;
        uptakeStoppedBySwitch = false;

        intakeController.setState(false);
        intakeController.update();

        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        // NOTE: Don't disable turret here!
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
        Pose2d finalPose = tankDrive.pinpointLocalizer.getPose();
        PoseStorage.currentPose = finalPose;

        telemetry.addLine("=== AUTON COMPLETE ===");
        telemetry.addData("Pose SAVED", "x=%.1f, y=%.1f, heading=%.1f°",
                finalPose.position.x,
                finalPose.position.y,
                Math.toDegrees(finalPose.heading.toDouble()));
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
            }
        }

        // Disable turret
        if (turretField != null) {
            turretField.disable();
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
                () -> opModeIsActive(),
                telemetry  // Pass telemetry for debug output!
        );
    }
}