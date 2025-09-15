package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.LoaderController;

@Config
@TeleOp(name = "Loader Test", group = "Test")
public class LoaderTest extends LinearOpMode {

    LoaderController loaderController;

    public static double TARGET_POWER = 0;

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
        loaderController = new LoaderController(this);

        waitForStart();

        while (opModeIsActive()) {

            if (!useDashBoard) {
                if (gamepad1.x && !isPressingX) {
                    isPressingX = true;
                    loaderController.loaderStop();
                } else if (!gamepad1.x && isPressingX) {
                    isPressingX = false;
                }

                if (gamepad1.y && !isPressingY) {
                    isPressingY = true;
                    loaderController.loaderFull();
                } else if (!gamepad1.y && isPressingY) {
                    isPressingY = false;
                }

                if (gamepad1.a && !isPressingA) {
                    isPressingA = true;
                    loaderController.loaderQuarter();
                } else if (!gamepad1.a && isPressingA) {
                    isPressingA = false;
                }

                if (gamepad1.b && !isPressingB) {
                    isPressingB = true;
                    loaderController.loaderHalf();
                } else if (!gamepad1.b && isPressingB) {
                    isPressingB = false;
                }

                if (gamepad1.dpad_up && !isPressingDpadUp) {
                    isPressingDpadUp = true;
                    loaderController.setLoaderPower(loaderController.getLoaderPower() + 0.01);
                } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                    isPressingDpadUp = false;
                }

                if (gamepad1.dpad_down && !isPressingDpadDown) {
                    isPressingDpadDown = true;
                    loaderController.setLoaderPower(loaderController.getLoaderPower() - 0.01);
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
                loaderController.setLoaderPower(TARGET_POWER);

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
