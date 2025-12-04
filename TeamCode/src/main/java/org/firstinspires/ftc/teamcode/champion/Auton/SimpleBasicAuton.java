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
    public static double DRIVE_POWER = 0.5;
    public static double TURN_POWER = 0.4;
    public static double INTAKE_POWER = 0.3;

    // Distance and timing parameters
    public static long BACKWARD_TIME_MS = 3000;      // Time to go back 40 inches
    public static long TURN_LEFT_TIME_MS = 1000;     // Time to turn 45 degrees left
    public static long FORWARD_TIME_MS = 1500;       // Time to go forward 20 inches
    public static long INTAKE_TIME_MS = 2000;        // Time to run intake
    public static long BACKWARD_RETURN_TIME_MS = 1500; // Time to go back 20 inches
    public static long TURN_RIGHT_TIME_MS = 1000;    // Time to turn 45 degrees right
    public static long SHOOT_TIME_MS = 2000;         // Time to run transfer/uptake

    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // Thread for continuous shooter PID
    private Thread shooterThread;
    private volatile boolean runShooter = false;

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addLine("=============================");
        telemetry.addLine("Ready to start!");
        telemetry.addLine("This will test all systems:");
        telemetry.addLine("- Drive backward");
        telemetry.addLine("- Turn left");
        telemetry.addLine("- Drive forward with intake");
        telemetry.addLine("- Drive backward");
        telemetry.addLine("- Turn right");
        telemetry.addLine("- Test transfer/uptake");
        telemetry.addLine("=============================");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        globalTimer.reset();

        // Start shooter and keep it running
        shooterController.setTargetRPM(CONSTANT_SHOOTER_RPM);
        shooterController.startShooting();
        startShooterThread();

        // Wait for shooter to warm up
        telemetry.addLine("Warming up shooter...");
        telemetry.update();
        sleep(1000);

        // Execute autonomous sequence
        executeAutonomousSequence();

        // Cleanup
        cleanup();

        telemetry.addLine("=============================");
        telemetry.addLine("✓ AUTONOMOUS COMPLETE");
        telemetry.addLine("=============================");
        telemetry.addData("Total Time", "%.1fs", globalTimer.seconds());
        telemetry.update();
    }

    private void initializeRobot() {
        telemetry.addLine("═══════════════════════");
        telemetry.addLine("  HARDWARE INITIALIZATION");
        telemetry.addLine("═══════════════════════");

        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
        driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);
        telemetry.addData("✓ Drive", "OK");

        // Initialize intake
        DcMotor intakeMotor = null;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            telemetry.addData("✓ Intake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Intake", "NOT FOUND");
        }
        intakeController = new NewIntakeController(intakeMotor);

        // Initialize transfer
        DcMotor transferMotor = null;
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            telemetry.addData("✓ Transfer", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Transfer", "NOT FOUND");
        }
        transferController = new NewTransferController(transferMotor);

        // Initialize uptake
        CRServo uptakeServo = null;
        try {
            uptakeServo = hardwareMap.get(CRServo.class, "uptake");
            telemetry.addData("✓ Uptake", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Uptake", "NOT FOUND");
        }
        uptakeController = new UptakeController(uptakeServo);

        // Initialize shooter
        DcMotor shooterMotor = null;
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            telemetry.addData("✓ Shooter", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Shooter", "NOT FOUND");
        }
        shooterController = new NewShooterController(shooterMotor);

        // Initialize ramp
        try {
            rampController = new NewRampController(this);
            rampController.setTargetAngle(CONSTANT_RAMP_ANGLE);
            telemetry.addData("✓ Ramp", "OK");
        } catch (Exception e) {
            telemetry.addData("✗ Ramp", "NOT FOUND");
        }

        telemetry.update();
        sleep(1000);
    }

    private void executeAutonomousSequence() {

        // Step 1: Go backward 40 inches
        telemetry.addLine("=============================");
        telemetry.addLine("Step 1: Going backward 40\"");
        telemetry.addLine("=============================");
        telemetry.update();

        driveBackward(DRIVE_POWER, BACKWARD_TIME_MS);
        sleep(500);  // Brief pause between movements

        // Step 2: Turn left 45 degrees
        telemetry.addLine("=============================");
        telemetry.addLine("Step 2: Turning left 45°");
        telemetry.addLine("=============================");
        telemetry.update();

        turnLeft(TURN_POWER, TURN_LEFT_TIME_MS);
        sleep(500);

        // Step 3: Go forward 20 inches with intake running
        telemetry.addLine("=============================");
        telemetry.addLine("Step 3: Forward 20\" + Intake");
        telemetry.addLine("=============================");
        telemetry.update();

        // Start intake
        intakeController.setState(true);
        intakeController.update();

        driveForward(INTAKE_POWER, FORWARD_TIME_MS);

        // Keep intake running for a bit
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < INTAKE_TIME_MS) {
            intakeController.update();
            telemetry.addLine("Intake running...");
            telemetry.addData("Time", "%.1fs", timer.seconds());
            telemetry.update();
            sleep(50);
        }

        // Stop intake
        intakeController.setState(false);
        intakeController.update();
        sleep(500);

        // Step 4: Go backward 20 inches
        telemetry.addLine("=============================");
        telemetry.addLine("Step 4: Going backward 20\"");
        telemetry.addLine("=============================");
        telemetry.update();

        driveBackward(DRIVE_POWER, BACKWARD_RETURN_TIME_MS);
        sleep(500);

        // Step 5: Turn right 45 degrees
        telemetry.addLine("=============================");
        telemetry.addLine("Step 5: Turning right 45°");
        telemetry.addLine("=============================");
        telemetry.update();

        turnRight(TURN_POWER, TURN_RIGHT_TIME_MS);
        sleep(500);

        // Step 6: Test transfer and uptake (shoot balls)
        telemetry.addLine("=============================");
        telemetry.addLine("Step 6: Testing Shoot System");
        telemetry.addLine("=============================");
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

            telemetry.addLine("🔥 SHOOTING");
            telemetry.addData("Shooter RPM", "%.0f / %.0f", currentRPM, targetRPM);
            telemetry.addData("Time", "%.1fs", timer.seconds());
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
    }

    // ========== BASIC MOVEMENT METHODS ==========

    private void driveForward(double power, long timeMs) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < timeMs) {
            driveController.tankDrive(power, power);

            telemetry.addLine("Driving Forward");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1fs / %.1fs",
                    timer.seconds(), timeMs / 1000.0);
            telemetry.update();

            sleep(20);
        }
        driveController.stopDrive();
    }

    private void driveBackward(double power, long timeMs) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < timeMs) {
            driveController.tankDrive(-power, -power);

            telemetry.addLine("Driving Backward");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1fs / %.1fs",
                    timer.seconds(), timeMs / 1000.0);
            telemetry.update();

            sleep(20);
        }
        driveController.stopDrive();
    }

    private void turnLeft(double power, long timeMs) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < timeMs) {
            driveController.tankDrive(-power, power);

            telemetry.addLine("Turning Left");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1fs / %.1fs",
                    timer.seconds(), timeMs / 1000.0);
            telemetry.update();

            sleep(20);
        }
        driveController.stopDrive();
    }

    private void turnRight(double power, long timeMs) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < timeMs) {
            driveController.tankDrive(power, -power);

            telemetry.addLine("Turning Right");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1fs / %.1fs",
                    timer.seconds(), timeMs / 1000.0);
            telemetry.update();

            sleep(20);
        }
        driveController.stopDrive();
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