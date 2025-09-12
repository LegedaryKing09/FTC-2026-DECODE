package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;

@Config
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    IntakeController intakeController;

    public static double TARGETPOWER = 0;

    boolean isPressingX = false;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingStart = false;
    boolean useDashBoard = false;

    @Override
    public void runOpMode() {
        intakeController = new IntakeController(this);

        waitForStart();

        while (opModeIsActive()) {

            if (!useDashBoard) {
                if (gamepad1.x && !isPressingX) {
                    isPressingX = true;
                    intakeController.intakeRest();
                } else if (!gamepad1.x && isPressingX) {
                    isPressingX = false;
                }

                if (gamepad1.y && !isPressingY) {
                    isPressingY = true;
                    intakeController.intakeFull();
                } else if (!gamepad1.y && isPressingY) {
                    isPressingY = false;
                }

                if (gamepad1.a && !isPressingA) {
                    isPressingA = true;
                    intakeController.intakeQuarter();
                } else if (!gamepad1.a && isPressingA) {
                    isPressingA = false;
                }

                if (gamepad1.b && !isPressingB) {
                    isPressingB = true;
                    intakeController.intakeHalf();
                } else if (!gamepad1.b && isPressingB) {
                    isPressingB = false;
                }

                if (gamepad1.dpad_up && !isPressingDpadUp) {
                    isPressingDpadUp = true;
                    intakeController.setRollerPower(intakeController.getRollerPower() + 0.01);
                } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                    isPressingDpadUp = false;
                }

                if (gamepad1.dpad_down && !isPressingDpadDown) {
                    isPressingDpadDown = true;
                    intakeController.setRollerPower(intakeController.getRollerPower() - 0.01);
                } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                    isPressingDpadDown = false;
                }

                if (gamepad1.start && !isPressingStart) {
                    isPressingStart = true;
                    useDashBoard = true;

                } else if (!gamepad1.start && isPressingStart) {
                    isPressingStart = false;
                }
            }

            if (useDashBoard) {
                intakeController.setRollerPower(TARGETPOWER);

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
