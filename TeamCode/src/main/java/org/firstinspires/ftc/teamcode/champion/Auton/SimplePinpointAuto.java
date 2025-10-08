package org.firstinspires.ftc.teamcode.champion.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.champion.controller.PinpointAutoController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Autonomous(name = "Simple Pinpoint Auto", group = "Autonomous")
public class SimplePinpointAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize controllers
        SixWheelDriveController driveController = new SixWheelDriveController(this);
        PinpointAutoController autoController = new PinpointAutoController(this, driveController);

        // Telemetry to confirm initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Perform autonomous actions
        autoController.moveForward(48.0); // Move forward 48 inches
        autoController.turnLeft(90.0);    // Turn 90 degrees left

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}