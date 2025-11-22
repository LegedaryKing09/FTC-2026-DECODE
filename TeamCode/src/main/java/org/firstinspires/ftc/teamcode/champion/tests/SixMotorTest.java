package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.SixMotorDriveController;

@Config
@TeleOp
public class SixMotorTest extends LinearOpMode {

    SixMotorDriveController driveController;

    @Override
    public void runOpMode() {
        driveController = new SixMotorDriveController(this);

        boolean isPressingB = false;
        boolean isPressingA = false;
        boolean isPressingY = false;
        boolean isPressingX = false;
        boolean isPressingLeftBumper = false;
        boolean isPressingRightBumper = false;
        boolean isPressingStart = false;
        boolean isPressingBack = false;

        double drive = -gamepad1.left_stick_y * SixMotorDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixMotorDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {
            if (driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixMotorDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixMotorDriveController.FAST_TURN_MULTIPLIER;
            }
            if (!driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixMotorDriveController.SLOW_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixMotorDriveController.SLOW_TURN_MULTIPLIER;
            }

            driveController.arcadeDrive(drive, turn);
//
//            if (gamepad1.a && !isPressingA) {
//                isPressingA = true;
//                driveController.setL1MotorsPowerOne();
//            } else if (!gamepad1.a && isPressingA) {
//                isPressingA = false;
//            }
//
//            if (gamepad1.b && !isPressingB) {
//                isPressingB = true;
//                driveController.setL2MotorsPowerOne();
//            } else if (!gamepad1.b && isPressingB) {
//                isPressingB = false;
//            }
//
//            if (gamepad1.x && !isPressingX) {
//                isPressingX = true;
//                driveController.setL3MotorsPowerOne();
//            } else if (!gamepad1.x && isPressingX) {
//                isPressingX = false;
//            }
//
//            if (gamepad1.y && !isPressingY) {
//                isPressingY = true;
//                driveController.setR1MotorsPowerOne();
//            } else if (!gamepad1.y && isPressingY) {
//                isPressingY = false;
//            }
//
//            if (gamepad1.left_bumper && !isPressingLeftBumper) {
//                isPressingLeftBumper = true;
//                driveController.setR2MotorsPowerOne();
//            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
//                isPressingLeftBumper = false;
//            }
//
//            if (gamepad1.right_bumper && !isPressingRightBumper) {
//                isPressingRightBumper = true;
//                driveController.setR3MotorsPowerOne();
//            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
//                isPressingRightBumper = false;
//            }
//
//            if (gamepad1.start && !isPressingStart) {
//                isPressingStart = true;
//                driveController.setMotorsPowerZero();
//            } else if (!gamepad1.start && isPressingStart) {
//                isPressingStart = false;
//            }
//
            if (gamepad1.back && !isPressingBack) {
                isPressingBack = true;
                driveController.toggleSpeedMode();
            } else if (!gamepad1.back && isPressingBack) {
                isPressingBack = false;
            }

            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            // Display encoder telemetry for odometry pods
            telemetry.addData("Speed Mode", driveController.isFastSpeedMode() ? "Fast" : "Slow");
            telemetry.addData("X Odometry Encoder", driveController.getXOdoPosition());
            telemetry.addData("Y Odometry Encoder", driveController.getYOdoPosition());
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Left Velocity", "%.0f ticks/s", driveController.getLeftVelocity());
            telemetry.addData("Right Velocity", "%.0f ticks/s", driveController.getRightVelocity());
            telemetry.update();
        }
    }
}