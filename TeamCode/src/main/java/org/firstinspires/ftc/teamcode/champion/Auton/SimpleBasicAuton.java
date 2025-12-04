package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.champion.controller.NewTransferController;
import org.firstinspires.ftc.teamcode.champion.controller.UptakeController;
import org.firstinspires.ftc.teamcode.champion.controller.NewShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.NewIntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.NewRampController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@Autonomous(name = "Simple Basic Auton", group = "Testing")
public class SimpleBasicAuton extends LinearOpMode {

    // Controllers
    SixWheelDriveController driveController;
    NewTransferController transferController;
    UptakeController uptakeController;
    NewShooterController shooterController;
    NewIntakeController intakeController;
    NewRampController rampController;

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
    private volatile boolean runShooter =  false;

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

        telemetry.update();
        sleep(1000);
    }

    private void executeAutonomousSequence() {

        // Step 1: Go backward 40 inches (using encoders)

        driveDistance(-BACKWARD_DISTANCE, DRIVE_POWER);
        sleep(500);  // Brief pause between movements

        // Step 2: Turn left 45 degrees (using encoders)

        turnAngle(-TURN_LEFT_ANGLE, TURN_POWER);
        sleep(500);

        // Step 3: Go forward 20 inches with intake running (using encoders)
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

        driveDistance(-BACKWARD_RETURN_DISTANCE, DRIVE_POWER);
        sleep(500);

        // Step 5: Turn right 45 degrees (using encoders)

        turnAngle(TURN_RIGHT_ANGLE, TURN_POWER);
        sleep(500);

        // Step 6: Test transfer and uptake (shoot balls)
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

            sleep(50);
        }

        // Stop all shooting systems
        transferController.setState(false);
        transferController.update();

        uptakeController.setState(false);
        uptakeController.update();

        intakeController.setState(false);
        intakeController.update();
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

            // Apply power in the correct direction
            driveController.tankDrive(power * direction, power * direction);


            sleep(20);
        }

        driveController.stopDrive();
        sleep(100);  // Brief settling time
    }

    private void turnAngle(double angleDegrees, double power) {
        // Update odometry to get starting heading
        driveController.updateOdometry();
        double startHeading = driveController.getHeadingDegrees();
        double targetHeading = startHeading + angleDegrees;

        // Normalize target heading to -180 to 180
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        // Determine turn direction
        double direction = Math.signum(angleDegrees);

        timer.reset();

        while (opModeIsActive()) {
            // Update odometry
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();

            // Calculate angle turned
            double angleTurned = Math.abs(currentHeading - startHeading);

            // Handle wraparound
            if (angleTurned > 180) {
                angleTurned = 360 - angleTurned;
            }

            // Check if we've reached the target
            if (angleTurned >= Math.abs(angleDegrees)) {
                break;
            }

            // Apply tank turn (positive = clockwise, negative = counterclockwise)
            driveController.tankDrive(power * direction, -power * direction);
            

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
    }
}