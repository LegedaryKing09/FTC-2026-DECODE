package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "LED Digital Test", group = "Test")
public class LEDDigitalTest extends OpMode {

    private DigitalChannel digitalPort;

    @Override
    public void init() {
        digitalPort = hardwareMap.get(DigitalChannel.class, "digital0");
        digitalPort.setMode(DigitalChannel.Mode.OUTPUT);
        digitalPort.setState(false);

        telemetry.addData("Status", "Initialized - Digital Port 0 set to OUTPUT");
        telemetry.addData("Controls", "A = HIGH | B = LOW");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            digitalPort.setState(true);
        } else if (gamepad1.b) {
            digitalPort.setState(false);
        }

        telemetry.addData("Digital Port 0", digitalPort.getState() ? "HIGH" : "LOW");
        telemetry.addData("Controls", "A = HIGH | B = LOW");
        telemetry.update();
    }

    @Override
    public void stop() {
        digitalPort.setState(false);
    }
}
