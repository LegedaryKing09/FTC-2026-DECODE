package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@TeleOp
public class BasicTeleopTest extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    public static double SHOOTING_POWER = 0;
    public static double INTAKE_POWER = 0;

    boolean isUsingTelemetry = true;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingX = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingDPADLeft = false;
    boolean isPressingStart = false;

    @Override
    public void runOpMode() {


        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {

            if(driveController.isFastSpeedMode()) {
                drive =-gamepad1.left_stick_y * SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.FAST_TURN_MULTIPLIER;
            }
            if(!driveController.isFastSpeedMode()) {
                drive =-gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
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
                intakeController.intakeFull();
                driveController.tankDrive(0.8,0.8);
                sleep(300);
                driveController.tankDrive(-0.8,-0.8);
                sleep(300);
                transferController.transferFull();
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

            if (gamepad1.dpad_up && SHOOTING_POWER < 1) {
                SHOOTING_POWER += 0.1;
            }

            if (gamepad1.dpad_down && SHOOTING_POWER > 0) {
                SHOOTING_POWER -= 0.1;
            }

            if (gamepad1.dpad_right && INTAKE_POWER < 1) {
                INTAKE_POWER += 0.1;
            }

            if (gamepad1.dpad_left && INTAKE_POWER > 0) {
                INTAKE_POWER -= 0.1;
            }

            if (gamepad1.right_trigger > 0.1) {
                transferController.transferFull();
            }

            if (gamepad1.right_trigger < 0.1) {
                transferController.transferStop();
            }

            if (gamepad1.left_trigger > 0.1) {
                transferController.transferEject();
            }

            if (gamepad1.left_trigger < 0.1) {
                transferController.transferStop();
            }

            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;
                intakeController.intakeFull();
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
                intakeController.intakeStop();
            }

            if (gamepad1.dpad_left && !isPressingDPADLeft) {
                isPressingDPADLeft = true;
                intakeController.intakeEject();
            } else if (!gamepad1.dpad_left && isPressingDPADLeft) {
                isPressingDPADLeft = false;
                intakeController.intakeStop();
            }

            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
            }

            if (gamepad1.start && !isPressingStart) {
                isPressingStart = true;
                isUsingTelemetry = !isUsingTelemetry;

            } else if (!gamepad1.start && isPressingStart) {
                isPressingStart = false;
            }

            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            if(isUsingTelemetry) {

                driveController.getMotorStatus();

                telemetry.addData("Expected Left Power", "%.2f", leftPower);
                telemetry.addData("Expected Right Power", "%.2f", rightPower);

                telemetry.addData("Shooting Power:", "%.2f", SHOOTING_POWER);
                telemetry.addData("Intake Power:", "%.2f", INTAKE_POWER);

                telemetry.addData("Robot X", "%.2f", driveController.getX());
                telemetry.addData("Robot Y", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

                telemetry.update();
            }

        }
    }
}