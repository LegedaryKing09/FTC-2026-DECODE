package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Odometry Tuner for Pinpoint + Six Wheel Drive
 *
 * Use this to calibrate:
 * 1. YAW_SCALAR - Heading accuracy (turn calibration)
 * 2. ENCODER_RESOLUTION - Distance accuracy
 * 3. POD_OFFSETS - X and Y offset from center
 *
 * TUNING PROCESS:
 *
 * === STEP 1: YAW SCALAR (Heading) ===
 * 1. Press A to reset position
 * 2. Manually rotate robot EXACTLY 10 full rotations (3600°) clockwise
 * 3. Check "Reported Heading" - it should show ~0° (back to start)
 * 4. If heading shows +X°: YAW_SCALAR is too HIGH, decrease it
 * 5. If heading shows -X°: YAW_SCALAR is too LOW, increase it
 * 6. Formula: NEW_SCALAR = OLD_SCALAR * (3600 / (3600 + reported_error))
 *
 * === STEP 2: DISTANCE (Encoder Resolution) ===
 * 1. Press A to reset position
 * 2. Push robot forward EXACTLY 100 inches (use tape measure)
 * 3. Check "Y Position" - it should show ~100 inches
 * 4. If Y shows 98 in: resolution too HIGH, decrease it
 * 5. If Y shows 102 in: resolution too LOW, increase it
 * 6. Formula: NEW_RES = OLD_RES * (measured_distance / reported_distance)
 *
 * === STEP 3: POD OFFSETS ===
 * 1. X_OFFSET: Distance from center to X (forward) pod (negative if behind center)
 * 2. Y_OFFSET: Distance from center to Y (strafe) pod (negative if left of center)
 * 3. Measure physically with ruler/calipers
 *
 * Controls:
 *   Left Stick = Drive (for testing)
 *   Right Stick = Turn (for testing)
 *   A = Reset odometry to (0,0,0)
 *   B = Start/Stop 10-spin test
 *   Y = Mark current position (for distance test)
 *   X = Calculate distance from marked position
 *   RB = Toggle speed mode
 */
@Config
@TeleOp(name = "Odometry Tuner", group = "Tuning")
public class OdometryTuner extends LinearOpMode {

    // ========== ODOMETRY PARAMETERS (tune in Dashboard) ==========
    public static double YAW_SCALAR = 1.0;           // Heading multiplier
    public static double ENCODER_RESOLUTION = 19.89; // mm per tick (4-bar pods default)
    public static double X_OFFSET_MM = -84.0;        // Forward pod offset from center
    public static double Y_OFFSET_MM = -168.0;       // Strafe pod offset from center
    public static boolean X_REVERSED = false;
    public static boolean Y_REVERSED = false;

    // Hardware
    private SixWheelDriveController drive;
    private GoBildaPinpointDriver pinpoint;

    // Test state
    private double markedX = 0, markedY = 0;
    private boolean testRunning = false;
    private double startHeading = 0;
    private int spinCount = 0;
    private double lastHeading = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive
        drive = new SixWheelDriveController(this);
        pinpoint = drive.getPinpoint();

        // Apply initial settings
        applyOdometrySettings();

        telemetry.addLine("=== ODOMETRY TUNER ===");
        telemetry.addLine();
        telemetry.addLine("A = Reset to (0,0,0)");
        telemetry.addLine("B = 10-spin test (heading)");
        telemetry.addLine("Y = Mark position");
        telemetry.addLine("X = Calc distance from mark");
        telemetry.addLine();
        telemetry.addLine("Tune values in Dashboard!");
        telemetry.update();

        waitForStart();

        boolean wasPressingA = false;
        boolean wasPressingB = false;
        boolean wasPressingY = false;
        boolean wasPressingX = false;

        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();
            pinpoint.update();

            // Get raw pose from pinpoint
            Pose2D pose = pinpoint.getPosition();
            double rawX = pose.getX(DistanceUnit.INCH);
            double rawY = pose.getY(DistanceUnit.INCH);
            double rawHeading = pose.getHeading(AngleUnit.DEGREES);

            // === DRIVE CONTROL ===
            double driveInput = -gamepad1.left_stick_y;
            double turnInput = gamepad1.right_stick_x;
            double speedMult = drive.isFastSpeedMode() ? 0.8 : 0.3;
            double turnMult = drive.isFastSpeedMode() ? 0.5 : 0.25;
            drive.arcadeDrive(driveInput * speedMult, turnInput * turnMult);

            // === BUTTON: A = Reset ===
            if (gamepad1.a && !wasPressingA) {
                drive.resetOdometry();
                spinCount = 0;
                testRunning = false;
                applyOdometrySettings();  // Re-apply settings after reset
            }
            wasPressingA = gamepad1.a;

            // === BUTTON: B = 10-Spin Test ===
            if (gamepad1.b && !wasPressingB) {
                if (!testRunning) {
                    // Start test
                    testRunning = true;
                    startHeading = rawHeading;
                    lastHeading = rawHeading;
                    spinCount = 0;
                } else {
                    // End test - calculate error
                    testRunning = false;
                    double totalRotation = (spinCount * 360) + (rawHeading - startHeading);
                    double error = totalRotation - 3600;  // Should be 3600° for 10 spins
                    double suggestedScalar = YAW_SCALAR * (3600.0 / totalRotation);

                    telemetry.addLine("=== 10-SPIN TEST RESULT ===");
                    telemetry.addData("Total Rotation", "%.1f°", totalRotation);
                    telemetry.addData("Expected", "3600°");
                    telemetry.addData("Error", "%.1f°", error);
                    telemetry.addData("Current YAW_SCALAR", "%.6f", YAW_SCALAR);
                    telemetry.addData("Suggested YAW_SCALAR", "%.6f", suggestedScalar);
                }
            }
            wasPressingB = gamepad1.b;

            // Track spins during test
            if (testRunning) {
                double headingDelta = rawHeading - lastHeading;
                // Detect wraparound
                if (headingDelta > 180) {
                    spinCount--;  // Crossed from positive to negative (CCW)
                } else if (headingDelta < -180) {
                    spinCount++;  // Crossed from negative to positive (CW)
                }
                lastHeading = rawHeading;
            }

            // === BUTTON: Y = Mark Position ===
            if (gamepad1.y && !wasPressingY) {
                markedX = rawX;
                markedY = rawY;
            }
            wasPressingY = gamepad1.y;

            // === BUTTON: X = Calculate Distance ===
            double distanceFromMark = 0;
            if (gamepad1.x && !wasPressingX) {
                double dx = rawX - markedX;
                double dy = rawY - markedY;
                distanceFromMark = Math.sqrt(dx*dx + dy*dy);
            }
            wasPressingX = gamepad1.x;

            // === BUTTON: RB = Speed Mode ===
            if (gamepad1.right_bumper) {
                drive.toggleSpeedMode();
                sleep(200);
            }

            // === TELEMETRY ===
            telemetry.addLine("=== CURRENT POSITION ===");
            telemetry.addData("X", "%.2f in", rawX);
            telemetry.addData("Y", "%.2f in", rawY);
            telemetry.addData("Heading", "%.2f°", rawHeading);
            telemetry.addLine();

            telemetry.addLine("=== RAW ENCODERS ===");
            telemetry.addData("X Encoder", drive.getXOdoPosition());
            telemetry.addData("Y Encoder", drive.getYOdoPosition());
            telemetry.addLine();

            telemetry.addLine("=== PINPOINT STATUS ===");
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Loop Time", "%d µs", pinpoint.getLoopTime());
            telemetry.addData("Frequency", "%.0f Hz", pinpoint.getFrequency());
            telemetry.addLine();

            telemetry.addLine("=== CURRENT SETTINGS ===");
            telemetry.addData("YAW_SCALAR", "%.6f", YAW_SCALAR);
            telemetry.addData("ENCODER_RES", "%.4f mm", ENCODER_RESOLUTION);
            telemetry.addData("X_OFFSET", "%.1f mm", X_OFFSET_MM);
            telemetry.addData("Y_OFFSET", "%.1f mm", Y_OFFSET_MM);
            telemetry.addLine();

            if (testRunning) {
                double currentRotation = (spinCount * 360) + (rawHeading - startHeading);
                telemetry.addLine("=== 10-SPIN TEST RUNNING ===");
                telemetry.addData("Spins Detected", spinCount);
                telemetry.addData("Total Rotation", "%.1f°", currentRotation);
                telemetry.addLine("Spin robot 10 times CW, then press B");
            }

            if (markedX != 0 || markedY != 0) {
                double dx = rawX - markedX;
                double dy = rawY - markedY;
                double dist = Math.sqrt(dx*dx + dy*dy);
                telemetry.addLine();
                telemetry.addLine("=== DISTANCE TEST ===");
                telemetry.addData("Marked Position", "(%.1f, %.1f)", markedX, markedY);
                telemetry.addData("Distance from Mark", "%.2f in", dist);
            }

            telemetry.addLine();
            telemetry.addData("Speed Mode", drive.isFastSpeedMode() ? "FAST" : "SLOW");

            telemetry.update();
        }

        drive.stopDrive();
    }

    private void applyOdometrySettings() {
        // Apply encoder directions
        GoBildaPinpointDriver.EncoderDirection xDir = X_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaPinpointDriver.EncoderDirection yDir = Y_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.REVERSED :
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        pinpoint.setEncoderDirections(xDir, yDir);
        pinpoint.setEncoderResolution((float) ENCODER_RESOLUTION, DistanceUnit.MM);
        pinpoint.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setYawScalar(YAW_SCALAR);
    }
}