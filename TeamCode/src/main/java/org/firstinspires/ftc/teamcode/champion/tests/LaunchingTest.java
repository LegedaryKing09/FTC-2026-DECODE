package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;

@Config
@TeleOp(name = "Launching Test", group = "Test")
public class LaunchingTest extends LinearOpMode {
    ShooterController shooterController;
    TransferController transferController;
    IntakeController intakeController;

    public static double SHOOT_TARGET_POWER = 0;
    public static double TRANSFER_TARGET_POWER = 0;

    boolean isPressingX = false;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingRightBumper = false;
    boolean isPressingStart = false;
    boolean useDashBoard = false;

    @Override
    public void runOpMode() {
        shooterController = new ShooterController(this);

        waitForStart();

        while (opModeIsActive()) {

            if (!useDashBoard) {//toggle between using only dashboard or computer
                if (gamepad1.x && !isPressingX) {//if x is pressed stop the shooter motor
                    isPressingX = true;
                    shooterController.shooterStop();
                } else if (!gamepad1.x && isPressingX) {
                    isPressingX = false;
                }

                if (gamepad1.y && !isPressingY) {//if y is pressed shooter motor full power
                    isPressingY = true;
                    shooterController.shooterFull();
                } else if (!gamepad1.y && isPressingY) {
                    isPressingY = false;
                }

                if (gamepad1.a && !isPressingA) {//if a is pressed shooter motor quarter power
                    isPressingA = true;
                    shooterController.shooterQuarter();
                } else if (!gamepad1.a && isPressingA) {
                    isPressingA = false;
                }

                if (gamepad1.b && !isPressingB) {//if b is pressed shooter motor half power
                    isPressingB = true;
                    shooterController.shooterHalf();
                } else if (!gamepad1.b && isPressingB) {
                    isPressingB = false;
                }

                if (gamepad1.dpad_up && !isPressingDpadUp) {//if dpad up is pressed shooter slightly increase power
                    isPressingDpadUp = true;
                    shooterController.setShooterPower(shooterController.getShooterPower() + 0.05);
                } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                    isPressingDpadUp = false;
                }

                if (gamepad1.dpad_down && !isPressingDpadDown) {//if dpad down is pressed shooter slightly decrease power
                    isPressingDpadDown = true;
                    shooterController.setShooterPower(shooterController.getShooterPower() - 0.05);
                } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                    isPressingDpadDown = false;
                }

                if (gamepad1.left_bumper) {//if left bumper is pressed loader at quarter full power
                    transferController.transferQuarter();//(change as needed)
                }

                if (!gamepad1.left_bumper) {// if stopped pressing loader stop
                    transferController.transferStop();
                }

                if (gamepad1.right_bumper && !isPressingRightBumper) {//if right is pressed intake motor half power
                    isPressingRightBumper = true;
                    intakeController.intakeHalf();//change as needed
                } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                    isPressingRightBumper = false;
                    intakeController.intakeRest();
                }


                if (gamepad1.start && !isPressingStart) {//dashboard toggle
                    isPressingStart = true;
                    useDashBoard = true;

                } else if (!gamepad1.start && isPressingStart) {
                    isPressingStart = false;
                }
            }

            if (useDashBoard) {//dashboard: get good powers from trial and error on computer
                shooterController.setShooterPower(SHOOT_TARGET_POWER);
                transferController.setTransferPower(TRANSFER_TARGET_POWER);

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
