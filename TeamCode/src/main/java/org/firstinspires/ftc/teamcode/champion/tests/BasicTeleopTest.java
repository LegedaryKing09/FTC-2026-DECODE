package org.firstinspires.ftc.teamcode.champion.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LoaderController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp
public class BasicTeleopTest extends LinearOpMode {

    SixWheelDriveController driveController;
    LoaderController loaderController;
    ShooterController shooterController;
    IntakeController intakeController;
    public static double SHOOTINGPOWER = 0;
    public static double INTAKEPOWER = 0;

    boolean isUsingTelemetry = true;
    boolean isPressingX = false;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingStart = false;

    @Override
    public void runOpMode() {


        driveController = new SixWheelDriveController(this);
        loaderController = new LoaderController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);

        double drive = -gamepad1.left_stick_y * driveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * driveController.SLOW_TURN_MULTIPLIER;

        waitForStart();

        while (opModeIsActive()) {

            if (!driveController.isFastSpeedMode()) {
                driveController.arcadeDrive(drive, turn);
            } else {
                driveController.arcadeDrive(drive, turn);
            }

            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                shooterController.shooterHalf();
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
                shooterController.shooterQuarter();
            }

            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                shooterController.shooterFull();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
                shooterController.shooterQuarter();
            }

            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                shooterController.setShooterPower(SHOOTINGPOWER);
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
                shooterController.shooterStop();
            }

            if (gamepad1.dpad_up && SHOOTINGPOWER < 1) {
                SHOOTINGPOWER += 0.1;
            }

            if (gamepad1.dpad_down && SHOOTINGPOWER > 0) {
                SHOOTINGPOWER -= 0.1;
            }

            if (gamepad1.dpad_right && INTAKEPOWER < 1) {
                INTAKEPOWER += 0.1;
            }

            if (gamepad1.dpad_left && INTAKEPOWER > 0) {
                INTAKEPOWER -= 0.1;
            }

            if (gamepad1.x) {
                loaderController.loaderFull();
            }

            if (!gamepad1.x) {
                loaderController.loaderStop();
            }

            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;
                intakeController.intakeHalf();
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
                intakeController.intakeRest();
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

                telemetry.addData("Shooting Power:", "%.2f", SHOOTINGPOWER);
                telemetry.addData("Intake Power:", "%.2f", INTAKEPOWER);

                telemetry.addData("Robot X", "%.2f", driveController.getX());
                telemetry.addData("Robot Y", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

                telemetry.update();
            }

        }
    }
}