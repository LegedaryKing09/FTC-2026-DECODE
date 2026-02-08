package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.LocalizationController;

/**
 * Test OpMode for LocalizationController
 * Displays robot position based on AprilTag detection
 *
 * Configure the AprilTag parameters in LocalizationController:
 * - TARGET_APRILTAG_ID (default: 20)
 * - APRILTAG_X (X coordinate of tag in inches)
 * - APRILTAG_Y (Y coordinate of tag in inches)
 */
@TeleOp(name = "Localization Test", group = "Test")
public class LocalizationTest extends LinearOpMode {

    private LocalizationController localization;

    // ========== CONFIGURE YOUR APRILTAG HERE ==========
    // These can also be changed via FTC Dashboard if using @Config
    private static final int APRILTAG_ID = 20;
    private static final double TAG_X = 0.0;  // inches
    private static final double TAG_Y = 0.0;  // inches

    @Override
    public void runOpMode() {
        // Initialize the localization controller
        localization = new LocalizationController(hardwareMap);

        // Set the AprilTag parameters
        localization.setTargetAprilTagId(APRILTAG_ID);
        localization.setAprilTagPosition(TAG_X, TAG_Y);

        // Show initialization status
        telemetry.addLine("=== Localization Test ===");
        telemetry.addData("Limelight", localization.isInitialized() ? "OK" : "NOT FOUND");
        telemetry.addData("Target Tag ID", APRILTAG_ID);
        telemetry.addData("Tag Position", "(%.1f, %.1f) inches", TAG_X, TAG_Y);
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get robot position
            LocalizationController.RobotPosition pos = localization.getRobotPosition();

            // === STATUS ===
            telemetry.addLine("=== Localization Status ===");
            telemetry.addData("Limelight", localization.isInitialized() ? "ACTIVE" : "ERROR");
            telemetry.addLine();

            // === APRILTAG CONFIG ===
            telemetry.addLine("=== AprilTag Config ===");
            telemetry.addData("Target ID", LocalizationController.TARGET_APRILTAG_ID);
            telemetry.addData("Tag Position", "(%.1f, %.1f) in",
                    LocalizationController.APRILTAG_X,
                    LocalizationController.APRILTAG_Y);
            telemetry.addLine();

            // === DETECTION STATUS ===
            telemetry.addLine("=== Detection ===");
            telemetry.addData("Target Tag Visible", localization.isTagVisible() ? "YES" : "NO");
            telemetry.addData("Total Tags Visible", localization.getVisibleTagCount());
            telemetry.addLine();

            // === ROBOT POSITION ===
            telemetry.addLine("=== Robot Position ===");
            if (pos.isValid) {
                telemetry.addData("X", "%.2f inches", pos.x);
                telemetry.addData("Y", "%.2f inches", pos.y);
                telemetry.addData("Status", "VALID");
            } else {
                telemetry.addData("X", "---");
                telemetry.addData("Y", "---");
                telemetry.addData("Status", "NO DATA (tag not visible)");
            }
            telemetry.addLine();

            // === DEBUG INFO ===
            telemetry.addLine("=== Debug ===");
            telemetry.addData("Info", localization.getDebugInfo());

            telemetry.update();
            sleep(50);
        }

        // Cleanup
        localization.shutdown();
    }
}
