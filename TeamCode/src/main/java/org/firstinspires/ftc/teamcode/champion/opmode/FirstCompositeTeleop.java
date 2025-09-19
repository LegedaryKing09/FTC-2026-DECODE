package org.firstinspires.ftc.teamcode.champion.opmode;

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
    LimelightAlignmentController alignmentController;
    public static double SHOOTING_POWER = 0.8;
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
            alignmentController = new LimelightAlignmentController(this);
        } catch (Exception e) {
            telemetry.addData("Limelight Alignment Error", e.getMessage());
            telemetry.addLine("Continuing without alignment controller...");
            alignmentController = null;
        }

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                intakeController.intakeFull();
                driveController.tankDrive(0.8, 0.8);
                sleep(300);
                driveController.tankDrive(-0.8, -0.8);
                sleep(300);
                driveController.stopDrive();
                intakeController.intakeStop();
                transferController.transferStop();
                shooterController.setShooterPower(SHOOTING_POWER);
                alignmentController.align(20);
                sleep(1000);
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
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            if (gamepad1.dpad_up && SHOOTING_POWER < 1 && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_POWER = SHOOTING_POWER + 0.05;
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            if (gamepad1.dpad_down && SHOOTING_POWER < 1 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_POWER = SHOOTING_POWER - 0.05;
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }
        }
    }
}