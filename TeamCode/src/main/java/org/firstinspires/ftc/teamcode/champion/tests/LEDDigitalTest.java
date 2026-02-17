package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(name = "LED Digital Test", group = "Test")
public class LEDDigitalTest extends OpMode {

    private LED led;

    @Override
    public void init() {
        led = hardwareMap.get(LED.class, "digital0");
        led.enableLight(false);

        telemetry.addData("Status", "Initialized - LED on Digital Port 0");
        telemetry.addData("Controls", "A = ON | B = OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            led.enableLight(true);
        } else if (gamepad1.b) {
            led.enableLight(false);
        }

        telemetry.addData("LED", led.isLightOn() ? "ON" : "OFF");
        telemetry.addData("Controls", "A = ON | B = OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        led.enableLight(false);
    }
}
