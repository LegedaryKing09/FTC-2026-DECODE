package org.firstinspires.ftc.teamcode.champion.Auton.drive.Q1Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "Red Far Auton", group = "Competition")
public class AutonFarRed extends LinearOpMode {
    SixWheelDriveController driveController;
    NewTransferController transferController;
    UptakeController uptakeController;
    NewShooterController shooterController;
    NewIntakeController intakeController;
    NewRampController rampController;
    LimelightAlignmentController limelightController;
    NewAutoShootController autoShootController;
    NewAutonController autonController;

    // Uptake ball detection switch
    private AnalogInput uptakeSwitch;
    public static double UPTAKE_SWITCH_THRESHOLD = 1.5;

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 5200.0;
    public static double CONSTANT_RAMP_ANGLE = 80.0;

    // Movement parameters (tunable via dashboard)
    public static double DRIVE_POWER = 0.5;
    public static double TURN_POWER = 0.3;
    public static double INTAKE_POWER = 0.35;

    // Distance parameters (in INCHES)
    public static double INITIAL_BACKWARD = 50.0;
    public static double LEFT_TURN_ANGLE = 50.0;
    public static double INTAKE_FORWARD = 18.0;
    public static double ENDING_DISTANCE = 24.0;
    public static double INTAKE_BACKWARD = 18.0;
    public static double FINAL_TURN_ANGLE = 45.0;
    public static double SHOOT_HEADING = 45.0;
    public static double PICK_UP_DISTANCE = 24.0;
    public static double SECOND_PICK_UP_DISTANCE = 48.0;

    // Timing parameters
    public static long INTAKE_TIME_MS = 2000;
    public static long SHOOT_TIME_MS = 4000;

    // Turning tolerance
    public static double TURN_TOLERANCE_DEGREES = 3.0;


    public boolean intakeModeActive = false;
    public boolean uptakeStoppedBySwitch = false;
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;

    @Override
    public void runOpMode() {
        initializeRobot();

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

        // Execute autonomous sequence
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
            telemetry.addData("Hardware Init Error", "Intake: " + e.getMessage());
        }
        intakeController = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Transfer: " + e.getMessage());
        }
        transferController = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake: " + e.getMessage());
        }
        uptakeController = new UptakeController(uptakeServo);

        // Initialize uptake ball detection switch
        try {
            uptakeSwitch = hardwareMap.get(AnalogInput.class, "uptakeSwitch");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Uptake Switch: " + e.getMessage());
        }

        // Initialize shooter
        DcMotor shooterMotorFirst = null;
        DcMotor shooterMotorSecond = null;
        try {
            shooterMotorFirst = hardwareMap.get(DcMotor.class, "shooter1");
            shooterMotorSecond = hardwareMap.get(DcMotor.class, "shooter2");
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Shooter: " + e.getMessage());
        }
        shooterController = new NewShooterController(shooterMotorFirst, shooterMotorSecond);

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", "Ramp: " + e.getMessage());
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
    }

    private void executeAutonomousSequence() {

        shootBalls();
        sleep(500);

        driveDistance(ENDING_DISTANCE, DRIVE_POWER);
        sleep(500);

        telemetry.addLine("COMPLETE!");
        telemetry.update();
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
        int ballsShotCount = 0;
        boolean lastBallState = false;

        while (opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            // Get current values for monitoring only
            boolean ballDetected = isBallAtUptake();
            boolean shooterReady = isShooterReady();
            double switchVoltage = (uptakeSwitch != null) ? uptakeSwitch.getVoltage() : -1.0;

            // Count balls shot (detect when ball passes through)
            if (ballDetected && !lastBallState) {
                ballsShotCount++;
            }
            lastBallState = ballDetected;

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

    private void intakeForward() {
        intakeModeActive = true;
        uptakeStoppedBySwitch = false;

        // Start all systems
        intakeController.setState(true);
        intakeController.update();

        transferController.setState(true);
        transferController.update();

        // Start uptake ON (will be controlled by switch during movement)
        uptakeController.setState(true);
        uptakeController.update();

        // Simple movement with switch checking DURING the drive
        driveController.updateOdometry();
        double startX = driveController.getX();

        timer.reset();

        while (opModeIsActive()) {
            driveController.updateOdometry();
            double distanceTraveled = Math.abs(driveController.getX() - startX);

            // Check if we've reached target
            if (distanceTraveled >= INTAKE_FORWARD) {
                break;
            }

            // Timeout safety (10 seconds)
            if (timer.seconds() > 5.0) {
                break;
            }

            // Drive forward at constant power
            driveController.tankDrive(INTAKE_POWER, INTAKE_POWER);

            checkUptakeSwitch();

            // Update all controllers
            intakeController.update();
            transferController.update();
            uptakeController.update();

            sleep(20);  // Check switch every 20ms
        }

        driveController.stopDrive();

        // Keep intake and transfer running for 2 more seconds after stopping
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            // Still check switch even after stopping
            checkUptakeSwitch();

            intakeController.update();
            transferController.update();
            uptakeController.update();
            sleep(50);
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
        if (uptakeSwitch == null) return true; // If no switch, assume ready
        return uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;
    }

    private boolean isShooterReady() {
        if (shooterController == null) return true;

        double currentRPM = shooterController.getRPM();
        double targetRPM = shooterController.getTargetRPM();
        double rpmTolerance = 100.0;

        return Math.abs(currentRPM - targetRPM) < rpmTolerance;
    }

    private void driveDistance(double distanceInches, double power) {
        // FIXED: Negate distance to correct reversed direction
        if (autonController != null) {
            autonController.moveRobot(-distanceInches, power);
        } else {
            // Fallback to simple odometry-based movement
            double direction = Math.signum(distanceInches);
            driveController.updateOdometry();
            double startX = driveController.getX();

            timer.reset();

            while (opModeIsActive()) {
                driveController.updateOdometry();
                double distanceTraveled = Math.abs(driveController.getX() - startX);

                if (distanceTraveled >= Math.abs(distanceInches)) {
                    break;
                }

                if (timer.seconds() > 10.0) {
                    break;
                }

                driveController.tankDrive(power * direction, power * direction);

                telemetry.addData("Target", "%.1f in", Math.abs(distanceInches));
                telemetry.addData("Current", "%.1f in", distanceTraveled);
                telemetry.update();

                sleep(20);
            }
            driveController.stopDrive();
            sleep(100);
        }
    }

    public void turnAngle(double angleDegrees, double power) {
        driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);

        driveController.updateOdometry();
        double startHeading = driveController.getHeadingDegrees();

        double kP = 0.02;
        double minPower = 0.15;
        timer.reset();

        while (opModeIsActive()) {
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();

            double turnedAngle = currentHeading - startHeading;
            while (turnedAngle > 180) turnedAngle -= 360;
            while (turnedAngle < -180) turnedAngle += 360;

            double error = angleDegrees - turnedAngle;

            if (Math.abs(error) < TURN_TOLERANCE_DEGREES) {
                break;
            }

            if (timer.seconds() > 5.0) {
                break;
            }

            double turnPower = error * kP;

            if (Math.abs(turnPower) > power) {
                turnPower = Math.signum(turnPower) * power;
            } else if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            driveController.tankDrive(-turnPower, turnPower);

            telemetry.addData("Target", "%.1f°", angleDegrees);
            telemetry.addData("Current", "%.1f°", turnedAngle);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Power", "%.2f", turnPower);
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();
        sleep(100);
    }

    public double getCurrentHeading() {
        driveController.updateOdometry();
        return Math.toDegrees(driveController.getHeading());
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
        driveController.stopDrive();

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

        // Ball detected when voltage is BELOW threshold (switch pressed)
        boolean ballDetected = uptakeSwitch.getVoltage() < UPTAKE_SWITCH_THRESHOLD;

        if (ballDetected && !uptakeStoppedBySwitch) {
            // Ball just arrived - STOP uptake (keep intake/transfer running)
            uptakeController.setState(false);
            uptakeController.update();
            uptakeStoppedBySwitch = true;
        } else if (!ballDetected && uptakeStoppedBySwitch) {
            // Ball removed/shot - RESTART uptake if still in intake mode
            uptakeController.setState(true);
            uptakeController.update();
            uptakeStoppedBySwitch = false;
        }
    }
}