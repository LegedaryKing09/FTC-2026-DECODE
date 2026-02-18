package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Two Servo Control", group = "TeleOp")
public class SingleServoOpmode extends LinearOpMode {

    private CRServo servo1;
    private CRServo servo2;

    @Override
    public void runOpMode() {
        // Initialize the servos
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("X", "Run Forward (Full Power)");
        telemetry.addData("Y", "Run Reverse (Full Power)");
        telemetry.addData("", "Release both for Float");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control servos with X and Y buttons
            if (gamepad1.x) {
                servo1.setPower(1.0);
                servo2.setPower(1.0);
            } else if (gamepad1.y) {
                servo1.setPower(-1.0);
                servo2.setPower(-1.0);
            } else {
                // Float state - servos stop but don't actively hold
                servo1.setPower(0.0);
                servo2.setPower(0.0);
            }

            // Display servo state
            telemetry.addData("Status", "Running");
            telemetry.addData("", "");
            telemetry.addData("Servo 1 Power", "%.2f", servo1.getPower());
            telemetry.addData("Servo 2 Power", "%.2f", servo2.getPower());
            telemetry.addData("", "");
            telemetry.addData("Button X", gamepad1.x ? "PRESSED (Forward)" : "");
            telemetry.addData("Button Y", gamepad1.y ? "PRESSED (Reverse)" : "");
            telemetry.addData("State", (!gamepad1.x && !gamepad1.y) ? "FLOAT" : "");
            telemetry.update();
        }

        // Stop servos when op mode ends
        servo1.setPower(0.0);
        servo2.setPower(0.0);
    }
}