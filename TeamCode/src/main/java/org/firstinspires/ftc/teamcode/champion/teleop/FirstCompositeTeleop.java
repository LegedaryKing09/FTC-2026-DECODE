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
public class FirstCompositeTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController LimelightAlignmentController;
    public static double SHOOTING_POWER = 0.65;
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
            LimelightAlignmentController = new LimelightAlignmentController(this);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                intakeController.intakeStop();
                transferController.transferStop();
                shooterController.setShooterPower(SHOOTING_POWER);

                // Fix: Start alignment before calling align
                LimelightAlignmentController.startAlignment();

                // Then continuously call align until aligned or timeout
                long alignmentStartTime = System.currentTimeMillis();
                long alignmentTimeout = 3000; // 3 seconds timeout

                while (opModeIsActive() && !LimelightAlignmentController.isAligned() &&
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
                sleep(2750);
                transferController.transferFull();
                intakeController.intakeFull();
                sleep(1500);
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

            if (gamepad1.dpad_up && SHOOTING_POWER < 1 && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_POWER = SHOOTING_POWER + 0.05;
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            if (gamepad1.dpad_down && SHOOTING_POWER > 0 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_POWER = SHOOTING_POWER - 0.05;
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }
        }
    }
}