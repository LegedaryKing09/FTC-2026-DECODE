package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PinpointAutoController {

    private final SixWheelDriveController driveController;
    private final LinearOpMode opMode;

    // Configurable powers
    private static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.3;
    private static final double POSITION_TOLERANCE_INCH = 1.0; // inches
    private static final double HEADING_TOLERANCE_DEG = 5.0; // degrees

    public PinpointAutoController(OpMode opMode, SixWheelDriveController driveController) {
        this.opMode = (LinearOpMode) opMode;
        this.driveController = driveController;
    }

    // Move forward by a specified distance in inches
    public void moveForward(double distanceInches) {
        double startX = driveController.getX(); // Get X in inches
        double targetX = startX + distanceInches;
        long startTime = System.currentTimeMillis();
        long timeoutMs = 10000; // 10 second timeout

        while (opMode.opModeIsActive() &&
                Math.abs(driveController.getX() - targetX) > POSITION_TOLERANCE_INCH &&
                (System.currentTimeMillis() - startTime) < timeoutMs) {
            driveController.tankDrive(DRIVE_POWER, DRIVE_POWER);
            driveController.updateOdometry();
            double currentX = driveController.getX();
            opMode.telemetry.addData("Start X", startX);
            opMode.telemetry.addData("Current X", currentX);
            opMode.telemetry.addData("Target X", targetX);
            opMode.telemetry.addData("Error", Math.abs(currentX - targetX));
            opMode.telemetry.update();
        }
        driveController.stopDrive();
        opMode.sleep(200); // Brief pause
    }

    // Turn left by a specified angle in degrees
    public void turnLeft(double degrees) {
        double startHeading = Math.toDegrees(driveController.getHeading());
        double targetHeading = startHeading + degrees;
        // Normalize to 0-360
        targetHeading = (targetHeading % 360 + 360) % 360;
        long startTime = System.currentTimeMillis();
        long timeoutMs = 10000; // 10 second timeout

        while (opMode.opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            double currentHeading = Math.toDegrees(driveController.getHeading());
            // Normalize current heading
            currentHeading = (currentHeading % 360 + 360) % 360;

            double error = targetHeading - currentHeading;
            // Handle wrap-around
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            if (Math.abs(error) <= HEADING_TOLERANCE_DEG) break;

            driveController.tankDrive(-TURN_POWER, TURN_POWER);
            driveController.updateOdometry();
            opMode.telemetry.addData("Start Heading", startHeading);
            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();
        }
        driveController.stopDrive();
        opMode.sleep(200); // Brief pause
    }

    // Move to a specific position (x, y) in inches relative to start
    public void moveToPosition(double xInches, double yInches) {
        // This would require more complex logic, perhaps using PID or simple proportional control
        // For now, just move in X and Y separately (simple implementation)
        moveForward(xInches); // Assuming starting at 0, move to X
        // For Y, would need strafe, but tank drive can't strafe easily
        // This is basic; for full field-relative movement, consider RoadRunner
    }
}