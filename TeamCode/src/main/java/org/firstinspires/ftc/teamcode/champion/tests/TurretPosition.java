package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Test OpMode to run turret servos individually to check for fighting.
 *
 * CONTROLS:
 * X = Servo 1 ONLY (disables servo 2)
 * Y = Servo 2 ONLY (disables servo 1)
 * A = BOTH servos enabled
 * B = FREE both (disable PWM)
 *
 * D-pad left/right = nudge position
 * D-pad up/down = bigger nudge (10x)
 */
@Config
@TeleOp(name = "Turret Single Servo Test", group = "Test")
public class TurretPosition extends LinearOpMode {

    public static String SERVO1_NAME = "turret1";
    public static String SERVO2_NAME = "turret2";
    public static double NUDGE_AMOUNT = 0.005;
    public static double BIG_NUDGE = 0.05;

    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, SERVO1_NAME);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO2_NAME);

        ServoController controller1 = servo1.getController();
        ServoController controller2 = servo2.getController();
        controller1.pwmDisable();
        controller2.pwmDisable();

        telemetry.addData("Status", "Both DISABLED - press X/Y/A to enable");
        telemetry.update();

        waitForStart();

        boolean lastX = false, lastY = false, lastA = false, lastB = false;
        boolean lastLeft = false, lastRight = false;
        boolean lastUp = false, lastDown = false;

        boolean servo1Enabled = false;
        boolean servo2Enabled = false;
        double servoPos = 0.5;

        while (opModeIsActive()) {
            // X = servo 1 only
            if (gamepad1.x && !lastX) {
                controller1.pwmEnable();
                controller2.pwmDisable();
                servo1.setPosition(servoPos);
                servo1Enabled = true;
                servo2Enabled = false;
            }
            lastX = gamepad1.x;

            // Y = servo 2 only
            if (gamepad1.y && !lastY) {
                controller1.pwmDisable();
                controller2.pwmEnable();
                servo2.setPosition(servoPos);
                servo1Enabled = false;
                servo2Enabled = true;
            }
            lastY = gamepad1.y;

            // A = both servos
            if (gamepad1.a && !lastA) {
                controller1.pwmEnable();
                controller2.pwmEnable();
                servo1.setPosition(servoPos);
                servo2.setPosition(servoPos);
                servo1Enabled = true;
                servo2Enabled = true;
            }
            lastA = gamepad1.a;

            // B = free both
            if (gamepad1.b && !lastB) {
                controller1.pwmDisable();
                controller2.pwmDisable();
                servo1Enabled = false;
                servo2Enabled = false;
            }
            lastB = gamepad1.b;

            // D-pad left/right = small nudge
            if (gamepad1.dpad_right && !lastRight) {
                servoPos = Math.min(1.0, servoPos + NUDGE_AMOUNT);
                if (servo1Enabled) servo1.setPosition(servoPos);
                if (servo2Enabled) servo2.setPosition(servoPos);
            }
            lastRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastLeft) {
                servoPos = Math.max(0.0, servoPos - NUDGE_AMOUNT);
                if (servo1Enabled) servo1.setPosition(servoPos);
                if (servo2Enabled) servo2.setPosition(servoPos);
            }
            lastLeft = gamepad1.dpad_left;

            // D-pad up/down = big nudge
            if (gamepad1.dpad_up && !lastUp) {
                servoPos = Math.min(1.0, servoPos + BIG_NUDGE);
                if (servo1Enabled) servo1.setPosition(servoPos);
                if (servo2Enabled) servo2.setPosition(servoPos);
            }
            lastUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDown) {
                servoPos = Math.max(0.0, servoPos - BIG_NUDGE);
                if (servo1Enabled) servo1.setPosition(servoPos);
                if (servo2Enabled) servo2.setPosition(servoPos);
            }
            lastDown = gamepad1.dpad_down;

            // Telemetry
            String mode;
            if (servo1Enabled && servo2Enabled) mode = "BOTH";
            else if (servo1Enabled) mode = "SERVO 1 ONLY";
            else if (servo2Enabled) mode = "SERVO 2 ONLY";
            else mode = "FREE";

            telemetry.addData("Mode", mode);
            telemetry.addData("Position", "%.4f", servoPos);
            telemetry.addLine();
            telemetry.addData("X", "Servo 1 only");
            telemetry.addData("Y", "Servo 2 only");
            telemetry.addData("A", "Both servos");
            telemetry.addData("B", "Free both");
            telemetry.addData("D-pad L/R", "Small nudge (%.3f)", NUDGE_AMOUNT);
            telemetry.addData("D-pad U/D", "Big nudge (%.3f)", BIG_NUDGE);
            telemetry.update();

            sleep(20);
        }

        controller1.pwmEnable();
        controller2.pwmEnable();
    }
}