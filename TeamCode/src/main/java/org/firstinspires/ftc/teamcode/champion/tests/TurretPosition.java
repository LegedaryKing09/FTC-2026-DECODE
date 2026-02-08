package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Test OpMode to find your turret zero position.
 *
 * HOW TO USE:
 * 1. Run this opmode – servos are disabled so turret moves freely
 * 2. Turn the turret by hand to your desired "0 degrees" position
 * 3. Press A to lock servos – they will hold current position
 * 4. Use D-pad left/right to nudge position in small increments
 * 5. Read "Servo Position" from telemetry
 * 6. Put that value into TurretController.ZERO_POSITION
 *
 * Press A to lock servos at mid position (0.5).
 * Press B to free servos again.
 * D-pad left/right to nudge position (while locked).
 */
@Config
@TeleOp(name = "Turret Zero Finder", group = "Test")
public class TurretPosition extends LinearOpMode {

    public static String SERVO1_NAME = "turret1";
    public static String SERVO2_NAME = "turret2";
    public static double NUDGE_AMOUNT = 0.005;

    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, SERVO1_NAME);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO2_NAME);

        ServoController controller1 = servo1.getController();
        ServoController controller2 = servo2.getController();
        controller1.pwmDisable();
        controller2.pwmDisable();

        telemetry.addData("Status", "Servos DISABLED - turn turret by hand");
        telemetry.update();

        waitForStart();

        boolean servosEnabled = false;
        boolean lastA = false;
        boolean lastB = false;
        boolean lastLeft = false;
        boolean lastRight = false;
        double servoPos = 0.5;  // Start at midpoint

        while (opModeIsActive()) {
            // A = lock servos at current position
            if (gamepad1.a && !lastA) {
                controller1.pwmEnable();
                controller2.pwmEnable();
                servo1.setPosition(servoPos);
                servo2.setPosition(servoPos);
                servosEnabled = true;
            }
            lastA = gamepad1.a;

            // B = free servos
            if (gamepad1.b && !lastB) {
                controller1.pwmDisable();
                controller2.pwmDisable();
                servosEnabled = false;
            }
            lastB = gamepad1.b;

            // D-pad nudge while locked
            if (servosEnabled) {
                if (gamepad1.dpad_right && !lastRight) {
                    servoPos = Math.min(1.0, servoPos + NUDGE_AMOUNT);
                    servo1.setPosition(servoPos);
                    servo2.setPosition(servoPos);
                }
                if (gamepad1.dpad_left && !lastLeft) {
                    servoPos = Math.max(0.0, servoPos - NUDGE_AMOUNT);
                    servo1.setPosition(servoPos);
                    servo2.setPosition(servoPos);
                }
            }
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            telemetry.addData("Servo Position", "%.4f", servoPos);
            telemetry.addData("Servos", servosEnabled ? "LOCKED" : "FREE");
            telemetry.addData("A = Lock", "B = Free");
            telemetry.addData("D-pad L/R", "Nudge position (when locked)");
            telemetry.addLine();
            telemetry.addData(">>> ZERO_POSITION", "%.4f", servoPos);
            telemetry.update();

            sleep(20);
        }

        controller1.pwmEnable();
        controller2.pwmEnable();
    }
}