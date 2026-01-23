package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.Auton.SimpleLimelightVision;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Comprehensive display of Limelight, Odometry, and Robot Center
 * Shows raw sensor data and calculated robot center positions
 */
@TeleOp(name = "Vision + Odometry + Center", group = "Test")
public class VisionOdometryTest extends LinearOpMode {

    private SimpleLimelightVision vision;
    private SixWheelDriveController drive;

    // Limelight offset from robot center (adjust this value for your robot)
    // Positive = Limelight is mounted forward of robot center
    private static final double LIMELIGHT_OFFSET_Y = 6.0; // inches

    @Override
    public void runOpMode() {
        // Initialize
        vision = new SimpleLimelightVision(hardwareMap);
        drive = new SixWheelDriveController(this);

        telemetry.addLine("Systems initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();

            // Get odometry position
            double odoX = drive.getX();
            double odoY = drive.getY();
            double odoH = drive.getHeadingDegrees();

            // === LIMELIGHT RAW DATA ===
            telemetry.addLine("Limelight");
            if (vision.hasValidData()) {
                telemetry.addData("Position", "X: %.1f\"  Y: %.1f\"",
                        vision.getX(), vision.getY());
                telemetry.addData("Heading", "%.1f° (Raw: %.1f°)",
                        vision.getHeading(), vision.getRawYaw());
                telemetry.addData("Tags Visible", vision.getTagCount());
            } else {
                telemetry.addLine("No AprilTags visible");
            }
            telemetry.addLine();

            // === ROBOT CENTER (from Limelight) ===
            telemetry.addLine("Center");
            if (vision.hasValidData()) {
                double centerX = vision.getRobotCenterX(LIMELIGHT_OFFSET_Y);
                double centerY = vision.getRobotCenterY(LIMELIGHT_OFFSET_Y);

                telemetry.addData("Position", "X: %.1f\"  Y: %.1f\"", centerX, centerY);
                telemetry.addData("Heading", "%.1f°", vision.getHeading());
            } else {
                telemetry.addLine("No vision data");
            }
            telemetry.addLine();

            // === ODOMETRY ===
            telemetry.addLine("Odometry");
            telemetry.addData("Position", "X: %.1f\"  Y: %.1f\"", odoX, odoY);
            telemetry.addData("Heading", "%.1f°", odoH);
            telemetry.addLine();

            telemetry.update();
            sleep(20);
        }

        vision.shutdown();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}