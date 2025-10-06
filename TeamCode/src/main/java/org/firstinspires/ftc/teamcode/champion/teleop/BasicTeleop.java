package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@TeleOp
public class BasicTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    public static double SHOOTING_POWER = 0;
    public static double INTAKE_POWER = 0;
    public static double TARGET_RPM = 2850;         // Target RPM for shooting
    public static int APRILTAG_ID = 20;             // AprilTag to align to
    public static double ALIGNMENT_THRESHOLD = 1.5; // Alignment error threshold in degrees
    public static long ALIGNMENT_TIMEOUT = 2000;    // Max time to wait for alignment (ms)
    public static long RPM_TIMEOUT = 3000;          // Max time to wait for RPM (ms)
    public static long SHOOT_DURATION = 1000;       // Duration to run transfer/intake (ms)
    public static long STABILITY_DELAY = 200;       // Delay after stopping alignment (ms)
    boolean isAutoShooting = false;
    boolean isManualAligning = false;
    boolean lastdpadLeft = false;
    boolean lastdpadRight = false;
    double lastShotTime = 0;
    int shotsCompleted = 0;
    boolean isUsingTelemetry = true;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingX = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingStart = false;
    boolean isPressingDpadDown = false;
    boolean isPressingDpadUp = false;
    boolean isPressingBack = false;
    private ElapsedTime sessionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            limelightController = new LimelightAlignmentController(this);
            limelightController.setTargetTag(APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {

            shooterController.updatePID();

            if (driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.FAST_TURN_MULTIPLIER;
            }
            if (!driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;
            }

            driveController.arcadeDrive(drive, turn);

            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                shooterController.shooterHalf();
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                shooterController.shooterFull();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                shooterController.setShooterPower(SHOOTING_POWER);
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                shooterController.shooterStop();
                intakeController.intakeStop();
                transferController.transferStop();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            if (gamepad1.dpad_up && SHOOTING_POWER < 1 && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_POWER = SHOOTING_POWER + 0.004;
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            if (gamepad1.dpad_down && SHOOTING_POWER > 0 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_POWER = SHOOTING_POWER - 0.004;
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }

            if (gamepad1.right_trigger > 0.1) {
                transferController.transferFull();
            } else if (gamepad1.left_trigger > 0.1) {
                transferController.transferEject();
            } else {
                transferController.transferStop();
            }

            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;
                intakeController.intakeFull();
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
                intakeController.intakeStop();
            }

            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;
                intakeController.intakeEject();
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
                intakeController.intakeStop();
            }

            if (gamepad1.back && !isPressingBack) {
                isPressingBack = true;
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
            } else if (!gamepad1.back && isPressingBack) {
                isPressingBack = false;
            }

            if (gamepad1.start && !isPressingStart) {
                isPressingStart = true;
                isUsingTelemetry = !isUsingTelemetry;

            } else if (!gamepad1.start && isPressingStart) {
                isPressingStart = false;
            }

            if (isManualAligning) {
                limelightController.align(APRILTAG_ID);
                if (limelightController.hasTarget() &&
                        limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            if (gamepad1.dpad_left && !lastdpadLeft && !isAutoShooting) {
                executeAutoShootSequence();
            }
            lastdpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !lastdpadRight) {
                if (!isManualAligning && !isAutoShooting) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }

            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            if (isUsingTelemetry) {

                driveController.getMotorStatus();

                telemetry.addData("Expected Left Power", "%.2f", leftPower);
                telemetry.addData("Expected Right Power", "%.2f", rightPower);

                telemetry.addData("Shooting Power:", SHOOTING_POWER);
                telemetry.addData("Intake Power:", INTAKE_POWER);

                telemetry.addData("Shooter Encoder Velocity(MPS):", shooterController.getShooterMPS());
                telemetry.addData("Shooter Encoder Velocity(RPM):", shooterController.getShooterRPM());
                telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
                telemetry.addData("Target RPM", "%.0f", shooterController.getTargetRPM());
                telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✓ YES" : "NO");
                //telemetry.addData("Is Fast Mode:", driveController.isFastSpeedMode());

                telemetry.addData("Robot X", "%.2f", driveController.getX());
                telemetry.addData("Robot Y", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());
            }

            telemetry.update();

        }
    }
    private void executeAutoShootSequence() {
        isAutoShooting = true;

        // Use a thread like the working version
        new Thread(() -> {
            try {
                // Step 1: Stop any existing movement
                driveController.stopDrive();

                // Step 2: Start spinning up shooter
                shooterController.setShooterRPM(TARGET_RPM);

                // Step 3: Wait for shooter to reach target RPM (with timeout)
                long rpmStartTime = System.currentTimeMillis();
                while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - rpmStartTime) < RPM_TIMEOUT) {
                    Thread.sleep(10);
                }

                // Step 4: Align to target (with error threshold check)
                if (limelightController != null) {
                    limelightController.startAlignment();

                    long alignStartTime = System.currentTimeMillis();
                    boolean alignmentGoodEnough = false;

                    // Keep aligning until within threshold OR timeout
                    while (opModeIsActive() &&
                            (System.currentTimeMillis() - alignStartTime) < ALIGNMENT_TIMEOUT) {

                        limelightController.align(APRILTAG_ID);

                        // Check if we're within acceptable error threshold
                        if (limelightController.hasTarget() &&
                                limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                            alignmentGoodEnough = true;
                            break;
                        }

                        Thread.sleep(20);
                    }

                    // Stop alignment and motors
                    limelightController.stopAlignment();
                    driveController.stopDrive();

                    // Brief stabilization delay
                    Thread.sleep(STABILITY_DELAY);
                }

                // Step 5: Execute the shot!
                // Ensure shooter is still at RPM
                if (shooterController.isAtTargetRPM() ||
                        Math.abs(shooterController.getRPMError()) < 200) {

                    // Run transfer and intake to shoot
                    transferController.transferFull();
                    intakeController.intakeFull();
                    Thread.sleep(SHOOT_DURATION);

                    // Stop transfer and intake
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotsCompleted++;
                    lastShotTime = sessionTimer.seconds();

                    telemetry.addLine(">>> SHOT COMPLETED <<<");
                    telemetry.addData("Shot #", shotsCompleted);
                }

                // Keep shooter running for next shot

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                isAutoShooting = false;
            }
        }).start();
    }
}