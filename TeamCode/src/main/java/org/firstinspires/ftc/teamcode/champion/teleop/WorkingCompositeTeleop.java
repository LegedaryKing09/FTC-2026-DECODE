package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@TeleOp
public class WorkingCompositeTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController LimelightAlignmentController;
    public static double SHOOTING_RPM = 2850;
    public static double RPM_INCREMENT = 100;
    public static double INTAKE_POWER = 0;

    boolean isUsingTelemetry = true;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingX = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingDpadLeft = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingStart = false;

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        try {
            LimelightAlignmentController = new LimelightAlignmentController(this, driveController);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {
            shooterController.updatePID();

            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                intakeController.intakeFull();
                driveController.tankDrive(0.8, 0.8);
                sleep(300);
                driveController.tankDrive(-0.8, -0.8);
                sleep(350);
                driveController.stopDrive();
                intakeController.intakeStop();
                transferController.transferStop();
                shooterController.setShooterRPM(SHOOTING_RPM);
                // Fix: Start alignment before calling align
                LimelightAlignmentController.startAlignment();

                // Then continuously call align until aligned or timeout
                long alignmentStartTime = System.currentTimeMillis();
                long alignmentTimeout = 3000; // 3 seconds timeout

                while (opModeIsActive() && LimelightAlignmentController.isAligned() &&
                        (System.currentTimeMillis() - alignmentStartTime) < alignmentTimeout) {
                    LimelightAlignmentController.align(20);
                    // Optional: Add telemetry to see alignment status
                    telemetry.addData("Alignment State", LimelightAlignmentController.getState());
                    telemetry.addData("Has Target", LimelightAlignmentController.hasTarget());
                    telemetry.addData("Error", LimelightAlignmentController.getTargetError());
                    telemetry.update();
                    sleep(20); // Small delay for smooth operation
                }

                // Stop alignment to prevent interference with subsequent operations
                LimelightAlignmentController.stopAlignment();
                driveController.stopDrive();
                shooterController.setShooterRPM(SHOOTING_RPM);

                long shooterStartTime = System.currentTimeMillis();
                long shooterTimeout = 3000; // 3 seconds max to reach RPM

                telemetry.addLine("Waiting for shooter to reach target RPM...");
                telemetry.update();

                while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - shooterStartTime) < shooterTimeout) {
                    // Keep updating PID while waiting
                    shooterController.updatePID();

                    telemetry.addData("Target RPM", "%.0f", shooterController.getTargetRPM());
                    telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
                    telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
                    telemetry.addData("At Target", shooterController.isAtTargetRPM());
                    telemetry.update();
                    sleep(10);
                }

                if (shooterController.isAtTargetRPM()) {
                    telemetry.addLine("Shooter at target RPM - FIRING!");
                    telemetry.update();

                    // Transfer and shoot
                    transferController.transferFull();
                    intakeController.intakeFull();

                    // Keep updating PID during shooting to maintain RPM
                    long shootingTime = System.currentTimeMillis();
                    while (opModeIsActive() && (System.currentTimeMillis() - shootingTime) < 1500) {
                        shooterController.updatePID();
                        sleep(10);
                    }
                } else {
                    telemetry.addLine("Failed to reach target RPM - aborting shot");
                    telemetry.update();
                    sleep(500);
                }
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                shooterController.shooterStop();
                intakeController.intakeStop();
                transferController.transferStop();
                // Also stop alignment when stopping everything
                LimelightAlignmentController.stopAlignment();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            if (gamepad1.dpad_up && SHOOTING_RPM < ShooterController.SHOOTER_FULL_RPM && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_RPM = Math.min(SHOOTING_RPM + RPM_INCREMENT, ShooterController.SHOOTER_FULL_RPM);
                telemetry.addData("Shooting RPM adjusted to", "%.0f", SHOOTING_RPM);
                telemetry.update();
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            if (gamepad1.dpad_down && SHOOTING_RPM > 0 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_RPM = Math.max(SHOOTING_RPM - RPM_INCREMENT, 0);
                telemetry.addData("Shooting RPM adjusted to", "%.0f", SHOOTING_RPM);
                telemetry.update();
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }
        }
    }
}