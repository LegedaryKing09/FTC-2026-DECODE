package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.NewAutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Autonomous(name = "Simple Basic Auton", group = "Testing")
public class SimpleBasicAuton extends LinearOpMode {
    // testing forward/backward movement, turning, intake, transfer, uptake, shooter 
    // later have to test limelight alignment, shooting, ramp adjustment according to the shooter power drop
    SixWheelDriveController driveController;
    NewTransferController transferController;
    UptakeController uptakeController;
    NewShooterController shooterController;
    NewIntakeController intakeController;
    NewRampController rampController;
    LimelightAlignmentController limelightController;
    NewAutoShootController autoShootController;

    // Shooter settings
    public static double CONSTANT_SHOOTER_RPM = 2700.0;
    public static double CONSTANT_RAMP_ANGLE = 119.0;

    // Movement parameters (tunable via dashboard)
    public static double DRIVE_POWER = 0.2;
    public static double TURN_POWER = 0.2;
    public static double INTAKE_POWER = 0.2;

    // Distance parameters (in INCHES)
    public static double BACKWARD_DISTANCE = 40.0;
    public static double TURN_LEFT_ANGLE = 45.0;
    public static double FORWARD_DISTANCE = 20.0;
    public static double BACKWARD_RETURN_DISTANCE = 20.0;
    public static double TURN_RIGHT_ANGLE = 45.0;

    // Timing parameters
    public static long INTAKE_TIME_MS = 2000;        // Time to run intake after driving forward
    public static long SHOOT_TIME_MS = 2000;         // Time to run transfer/uptake

    // Encoder constants (from SixWheelDriveController)
    private static final double TICKS_PER_REV = 751.8;
    private static final double WHEEL_DIAMETER_INCHES = 2.83;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Turning constants
    private static final double TRACK_WIDTH_INCHES = 11.5;
    private static final double DEGREES_TO_INCHES = (TRACK_WIDTH_INCHES * Math.PI) / 360.0;

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
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
        } catch (Exception e) {
            //
        }
        uptakeController = new UptakeController(uptakeServo);

        // Initialize shooter
        DcMotor shooterMotor = null;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        } catch (Exception e) {
            //
        }
        shooterController = new NewShooterController(shooterMotor);

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
        } catch (Exception e) {
            //
        }

        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(AutoShootController.APRILTAG_ID);
            autoShootController = new NewAutoShootController(this, driveController, shooterController,
                    intakeController, transferController, uptakeController, limelightController, rampController);
        } catch (Exception e) {
            limelightController = null;
            autoShootController = null;
        }

        telemetry.update();
        sleep(1000);
    }

    private void executeAutonomousSequence() {
        // Step 1: Go backward 40 inches (using encoders)
        telemetry.addLine("Step 1: Going backward");
        telemetry.update();
        driveDistance(-BACKWARD_DISTANCE, DRIVE_POWER);
        sleep(500);  // Brief pause between movements

        // Step 2: Turn left 45 degrees (using encoders)
        telemetry.addLine("Step 2: Turning left");
        telemetry.update();
        turnAngle(TURN_LEFT_ANGLE, TURN_POWER);
        sleep(500);

        // Step 3: Go forward 20 inches with intake running (using encoders)
        telemetry.addLine("Step 3: Going forward with intake");
        telemetry.update();

        // Start intake
        intakeController.setState(true);
        intakeController.update();

        driveDistance(FORWARD_DISTANCE, INTAKE_POWER);

        // Keep intake running for a bit
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            intakeController.update();
            sleep(50);
        }

        // Stop intake
        intakeController.setState(false);
        intakeController.update();
        sleep(500);

        // Step 4: Go backward 20 inches (using encoders)
        telemetry.addLine("Step 4: Going backward");
        telemetry.update();
        driveDistance(-BACKWARD_RETURN_DISTANCE, DRIVE_POWER);
        sleep(500);

        // Step 5: Turn right 45 degrees (using encoders)
        telemetry.addLine("Step 5: Turning right");
        telemetry.update();
        turnAngle(-TURN_RIGHT_ANGLE, TURN_POWER);  // Positive = right (clockwise)
        sleep(500);

        // Step 6: Test transfer and uptake (shoot balls)
        telemetry.addLine("Step 6: Shooting");
        telemetry.update();

        // Start transfer and uptake
        transferController.setState(true);
        transferController.update();

        uptakeController.setState(true);
        uptakeController.update();

        intakeController.setState(true);
        intakeController.update();

        // Run for specified time
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < SHOOT_TIME_MS) {
            transferController.update();
            uptakeController.update();
            intakeController.update();

            double currentRPM = shooterController.getRPM();
            double targetRPM = shooterController.getTargetRPM();

            telemetry.addData("RPM", "%.0f / %.0f", currentRPM, targetRPM);
            telemetry.update();

            sleep(50);
        }

        // Stop all shooting systems
        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        intakeController.setState(false);
        intakeController.update();

        telemetry.addLine("COMPLETE!");
        telemetry.update();
    }

    // ========== ENCODER-BASED MOVEMENT METHODS ==========

    private void driveDistance(double distanceInches, double power) {
        // Calculate target encoder ticks
        int targetTicks = (int) (Math.abs(distanceInches) * TICKS_PER_INCH);

        // Determine direction
        double direction = Math.signum(distanceInches);

        // Update odometry to get starting position
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startY = driveController.getY();

        timer.reset();

        while (opModeIsActive()) {
            // Update odometry
            driveController.updateOdometry();

            // Calculate distance traveled
            double deltaX = driveController.getX() - startX;
            double deltaY = driveController.getY() - startY;
            double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Check if we've reached the target
            if (distanceTraveled >= Math.abs(distanceInches)) {
                break;
            }

            // Timeout check (10 seconds)
            if (timer.seconds() > 10.0) {
                telemetry.addLine("⚠️ Drive timeout!");
                telemetry.update();
                break;
            }

            // Apply power in the correct direction
            driveController.tankDrive(power * direction, power * direction);

            // Show progress
            telemetry.addData("Target", "%.1f in", Math.abs(distanceInches));
            telemetry.addData("Current", "%.1f in", distanceTraveled);
            telemetry.addData("Heading", "%.1f°", driveController.getHeadingDegrees());
            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
        sleep(100);  // Brief settling time
    }
    private void turnAngle(double angleDegrees, double power) {
        // Update odometry to get starting heading
        double currentHeading = driveController.getHeadingDegrees();;
        boolean targetIsGreater = false;
        driveController.updateOdometry();
        double startHeading = driveController.getHeadingDegrees();
        double targetHeading = startHeading + angleDegrees;

        // Normalize target heading to -180 to 180
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        timer.reset();

        if(targetHeading > currentHeading) {
            targetIsGreater = true;
        } else {
            targetIsGreater = false;
        }

        while (targetIsGreater && targetHeading >= currentHeading || !targetIsGreater && targetHeading <= currentHeading ) {
            currentHeading = driveController.getHeadingDegrees();
            // Update odometry
            driveController.updateOdometry();

            double error = targetHeading - currentHeading;

            // Normalize error to -180 to 180 (shortest path)
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) < 3.0) {
                break;
            }

            // Timeout check (5 seconds)
            if (timer.seconds() > 5.0) {
                telemetry.addLine("⚠️ Turn timeout!");
                telemetry.update();
                break;
            }

            double turnPower = Math.signum(error) * power;

            driveController.tankDrive(-turnPower, turnPower);

            // Show progress
            telemetry.addData("Target", "%.1f°", targetHeading);
            telemetry.addData("Current", "%.1f°", currentHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.update();

            sleep(20);

        }

        driveController.stopDrive();
        sleep(100);  // Brief settling time
    }

    // ========== SHOOTER THREAD ==========

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
        // Stop shooter thread
        runShooter = false;
        if (shooterThread != null) {
            try {
                shooterThread.interrupt();
                shooterThread.join(500);
            } catch (Exception e) {
                // Ignore
            }
        }

        // Stop all systems
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
}