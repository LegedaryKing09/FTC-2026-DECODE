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
@Autonomous(name = "New Pathing for CB", group = "Test")
public class NewPathingForCB extends LinearOpMode {
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
    public static double AUTON_START_HEADING = 45.0;

    // ========

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 3000.0;
    public static double CONSTANT_RAMP_ANGLE = 0.0;
    // Distance parameters
    public static double INITIAL_BACKWARD = 30.0;
    public static double BACKWARD_Y = -25.0;
    public static double SPLINE_ANGLE = -60.0;
    public static double SECOND_INTAKE_DISTANCE = 38.0;
    public static double THIRD_INTAKE_DISTANCE = 60.0;
    public static double SPLINE_X = 0.0;
    public static double SPLINE_Y = -50.0;
    public static double SECOND_SPLINE_X = 45.0;
    public static double SECOND_SPLINE_Y = -60.0;
    public static double SECOND_SPLINE_ANGLE = -60.0;
    public static double SECOND_COMEBACK_POSITION = SPLINE_X + 5.0;
    public static double THIRD_COMEBACK_POSITION = SPLINE_X + 8.0;
    public static double SECOND_FORWARD= 24.0;
    public static double ENDING_DISTANCE = 30.0;

    // turning angle parameters
    public static double DEGREE_ZERO = 0.0;
    public static double PICK_UP_ANGLE = 90.0;
    public static double DEGREE_45 = 45.0;

    // turning perfection
    public static double HEADING_CORRECTION_KP = 0.005;
    public static double HEADING_CORRECTION_MAX_VEL = 0.16;
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
    public static double AUTO_AIM_LEFT = 0;

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

        // GO BACK FOR SHOOTING
        Action Initial_Forward = tankDrive.actionBuilder(currentPose)
                .lineToX(INITIAL_BACKWARD)
                .build();
        Actions.runBlocking(Initial_Forward);

        // SHOOT
        shootBalls();

        // SPLINE FOR INTAKE
        intakeSpline(INITIAL_BACKWARD, SPLINE_Y, SPLINE_ANGLE);

        currentPose = tankDrive.pinpointLocalizer.getPose();
        Action Backward = tankDrive.actionBuilder(currentPose)
                .lineToY(BACKWARD_Y)
                .build();
        Actions.runBlocking(Backward);

        shootBalls();

        currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action SPLINE_2 = tankDrive.actionBuilder(currentPose)
                .splineTo(new Vector2d(SECOND_SPLINE_X, SECOND_SPLINE_Y), Math.toRadians(SECOND_SPLINE_ANGLE))
                .build();
        Actions.runBlocking(SPLINE_2);

//        // 3. Go for Intake
//        intakeForwardRoadRunner(FIRST_INTAKE_DISTANCE);
//        currentPose = tankDrive.pinpointLocalizer.getPose();

//        // 4. Go Backward
//        Action Backward1 = tankDrive.actionBuilder(currentPose)
//                .lineToX()
//                .build();
//        Actions.runBlocking(Backward1);
//
//        // 5. shooting
//        shootBalls();
//
//
//        // 3. Go for Intake
//        intakeForwardRoadRunner(SECOND_INTAKE_DISTANCE);
//        currentPose = tankDrive.pinpointLocalizer.getPose();
//
//        // 4. Go Backward
//        Action Backward2 = tankDrive.actionBuilder(currentPose)
//                .lineToX(SECOND_COMEBACK_POSITION)
//                .build();
//        Actions.runBlocking(Backward2);
//
//        // 5. shooting
//        shootBalls();
//
//        // 6. Go for Intake
//        intakeForwardRoadRunner(THIRD_INTAKE_DISTANCE);
//        currentPose = tankDrive.pinpointLocalizer.getPose();
//
//        // 7. Go Backward
//        Action Backward3 = tankDrive.actionBuilder(currentPose)
//                .lineToX(THIRD_COMEBACK_POSITION)
//                .build();
//        Actions.runBlocking(Backward3);
//
//        // 8. shooting
//        shootBalls();
//
//        currentPose = tankDrive.pinpointLocalizer.getPose();
//        // exit out
//        Action Forward = tankDrive.actionBuilder(currentPose)
//                .lineToX(SPLINE_X + 20)
//                .build();
//        Actions.runBlocking(Forward);

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


    private void intakeSpline(double splineX, double splineY, double splineAngle) {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        // Get current pose and build trajectory
        Pose2d currentPose = tankDrive.pinpointLocalizer.getPose();
        // 1. Spline
        Action moveForward = tankDrive.actionBuilder(currentPose)
                .splineTo(new Vector2d(INITIAL_BACKWARD, SPLINE_Y), Math.toRadians(SPLINE_ANGLE))
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
        double fieldX = AUTON_START_X;
        double fieldY = AUTON_START_Y;
        double fieldHeading = Math.toRadians(AUTON_START_HEADING);
        double rawX = 0, rawY = 0;
        double deltaX = 0, deltaY = 0;
        double deltaHeading = 0;

        try {
            // Try to get pose from RoadRunner's pinpoint localizer
            Pose2d rawPose = tankDrive.pinpointLocalizer.getPose();
            rawX = rawPose.position.x;
            rawY = rawPose.position.y;
            deltaHeading = rawPose.heading.toDouble();  // This is delta from start (0)

            // === COORDINATE TRANSFORMATION ===
            // SWAP_XY = true, NEGATE_X = false, NEGATE_Y = true
            deltaX = rawY;       // Swapped, not negated
            deltaY = -rawX;      // Swapped, then negated

            // Add delta to starting position
            fieldX = AUTON_START_X + deltaX;
            fieldY = AUTON_START_Y + deltaY;
            fieldHeading = Math.toRadians(AUTON_START_HEADING) + deltaHeading;  // Add starting heading

        } catch (Exception e) {
            // If RoadRunner fails, try the SixWheelDriveController odometry
            telemetry.addData("WARNING", "RoadRunner getPose failed: " + e.getMessage());
            try {
                if (driveController != null) {
                    driveController.updateOdometry();
                    rawX = driveController.getX();
                    rawY = driveController.getY();
                    deltaHeading = driveController.getHeading();

                    // Same transformation
                    deltaX = rawY;
                    deltaY = -rawX;
                    fieldX = AUTON_START_X + deltaX;
                    fieldY = AUTON_START_Y + deltaY;
                    fieldHeading = Math.toRadians(AUTON_START_HEADING) + deltaHeading;
                }
            } catch (Exception e2) {
                telemetry.addData("WARNING", "DriveController also failed: " + e2.getMessage());
                // Use starting position as fallback
            }
        }

        // Save to PoseStorage (even if we only have starting position)
        try {
            PoseStorage.currentPose = new Pose2d(
                    new Vector2d(fieldX, fieldY),
                    fieldHeading
            );
        } catch (Exception e) {
            telemetry.addData("WARNING", "Pose2d creation failed: " + e.getMessage());
        }

        // Save turret angle
        if (turret != null) {
            try {
                PoseStorage.turretAngle = turret.getTurretAngle();
            } catch (Exception e) {
                PoseStorage.turretAngle = 0;
            }
        }
        telemetry.addData("Field Pose SAVED", "x=%.1f, y=%.1f, h=%.1f°",
                fieldX, fieldY, Math.toDegrees(fieldHeading));
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