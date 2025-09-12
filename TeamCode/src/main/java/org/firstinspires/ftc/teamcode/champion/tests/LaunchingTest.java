package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LoaderController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;

@Config
@TeleOp(name = "Launching Test", group = "Test")
public class LaunchingTest extends LinearOpMode {
    ShooterController shooterController;
    LoaderController loaderController;
    IntakeController intakeController;

    public static double SHOOTTARGETPOWER = 0;
    public static double LOADTARGETPOWER = 0;

    boolean isPressingX = false;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingStart = false;
    boolean useDashBoard = false;

    @Override
    public void runOpMode() {
        shooterController = new ShooterController(this);

        waitForStart();

        while (opModeIsActive()) {

            if (!useDashBoard) {
                if (gamepad1.x && !isPressingX) {
                    isPressingX = true;
                    shooterController.shooterStop();
                } else if (!gamepad1.x && isPressingX) {
                    isPressingX = false;
                }

                if (gamepad1.y && !isPressingY) {
                    isPressingY = true;
                    shooterController.shooterFull();
                } else if (!gamepad1.y && isPressingY) {
                    isPressingY = false;
                }

                if (gamepad1.a && !isPressingA) {
                    isPressingA = true;
                    shooterController.shooterQuarter();
                } else if (!gamepad1.a && isPressingA) {
                    isPressingA = false;
                }

                if (gamepad1.b && !isPressingB) {
                    isPressingB = true;
                    shooterController.shooterHalf();
                } else if (!gamepad1.b && isPressingB) {
                    isPressingB = false;
                }

                if (gamepad1.dpad_up && !isPressingDpadUp) {
                    isPressingDpadUp = true;
                    shooterController.setShooterPower(shooterController.getShooterPower() + 0.01);
                } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                    isPressingDpadUp = false;
                }

                if (gamepad1.dpad_down && !isPressingDpadDown) {
                    isPressingDpadDown = true;
                    shooterController.setShooterPower(shooterController.getShooterPower() - 0.01);
                } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                    isPressingDpadDown = false;
                }

                if (gamepad1.left_bumper) {
                    loaderController.loaderQuarter();
                }

                if (!gamepad1.left_bumper) {
                    loaderController.loaderStop();
                }

                if (gamepad1.right_bumper && !isPressingRightBumper) {
                    isPressingRightBumper = true;
                    intakeController.intakeHalf();
                } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                    isPressingRightBumper = false;
                    intakeController.intakeRest();
                }


                if (gamepad1.start && !isPressingStart) {
                    isPressingStart = true;
                    useDashBoard = true;

                } else if (!gamepad1.start && isPressingStart) {
                    isPressingStart = false;
                }
            }

            if (useDashBoard) {
                shooterController.setShooterPower(SHOOTTARGETPOWER);
                loaderController.setLoaderPower(LOADTARGETPOWER);

                if (gamepad1.start && !isPressingStart) {
                    isPressingStart = true;
                    useDashBoard = false;

                } else if (!gamepad1.start && isPressingStart) {
                    isPressingStart = false;
                }
            }


        }
    }
}
