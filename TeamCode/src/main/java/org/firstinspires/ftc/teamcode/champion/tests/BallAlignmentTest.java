package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SimpleBallAlignmentController;

/**
 * Ball Auto Alignment Test - Follows proven AprilTag alignment pattern
 * Press START to begin automatic ball tracking
 */
@TeleOp(name = "Ball Auto Align Test", group = "Test")
public class BallAlignmentTest extends LinearOpMode {

    private SimpleBallAlignmentController ballController;
    private ElapsedTime runtime = new ElapsedTime();

    // Performance tracking
    private int ballsTracked = 0;
    private int greenBalls = 0;
    private int purpleBalls = 0;
    private int lastSeenColor = 0;
    private ElapsedTime colorChangeTimer = new ElapsedTime();
    private ElapsedTime alignmentTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Ball Tracking...");
        telemetry.update();

        // Initialize controller
        try {
            ballController = new SimpleBallAlignmentController(this);
            telemetry.addLine("✓ Ball Tracking Initialized");
            telemetry.addLine("✓ Limelight Started");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Initialization failed: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // Display startup info
        telemetry.addLine();
        telemetry.addLine("╔═══════ BALL AUTO TRACKING ═══════╗");
        telemetry.addLine("║ Automatic ball alignment  ║");
        telemetry.addLine("║ Uses same PID as AprilTag ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("──── KEY PARAMETERS ────");
        telemetry.addData("KP", SimpleBallAlignmentController.BallAlignPID.KP);
        telemetry.addData("KD", SimpleBallAlignmentController.BallAlignPID.KD);
        telemetry.addData("Tolerance", SimpleBallAlignmentController.BallAlignZones.TOLERANCE + "°");
        telemetry.addData("Search", SimpleBallAlignmentController.BallAlignSearch.ENABLE_SEARCH ? "ON" : "OFF");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        runtime.reset();
        colorChangeTimer.reset();
        alignmentTimer.reset();

        // Start ball tracking
        ballController.startTracking();

        // Main loop - continuously call align() like AprilTag controller
        while (opModeIsActive()) {
            // Main alignment logic - same pattern as AprilTag
            ballController.align();

            // Track performance statistics
            trackPerformance();

            // Display comprehensive telemetry
            displayTelemetry();
            telemetry.update();

            // Standard FTC loop timing
            sleep(10);
        }

        // Cleanup
        ballController.stopTracking();
    }

    private void trackPerformance() {
        if (ballController.hasBall()) {
            int currentColor = ballController.getBallColor();

            // Track new balls (color changes)
            if (currentColor != lastSeenColor && colorChangeTimer.seconds() > 1.0) {
                ballsTracked++;
                if (currentColor == 1) {
                    greenBalls++;
                } else if (currentColor == 2) {
                    purpleBalls++;
                }
                lastSeenColor = currentColor;
                colorChangeTimer.reset();
            }

            // Track alignment time
            if (ballController.isAligned()) {
                // Ball is aligned
            }
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("╔═══════ BALL TRACKING ═══════╗");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("State", ballController.getState());
        telemetry.addLine();

        // Ball detection status
        ballController.displayTelemetry();

        // PID Configuration
        telemetry.addLine();
        telemetry.addLine("──── PID CONFIG ────");
        telemetry.addData("KP", "%.4f", SimpleBallAlignmentController.BallAlignPID.KP);
        telemetry.addData("KI", "%.4f", SimpleBallAlignmentController.BallAlignPID.KI);
        telemetry.addData("KD", "%.4f", SimpleBallAlignmentController.BallAlignPID.KD);
        telemetry.addData("Tolerance", "%.1f°", SimpleBallAlignmentController.BallAlignZones.TOLERANCE);

        // Statistics
        if (ballsTracked > 0) {
            telemetry.addLine();
            telemetry.addLine("──── STATISTICS ────");
            telemetry.addData("Total Balls", ballsTracked);
            telemetry.addData("Green", greenBalls);
            telemetry.addData("Purple", purpleBalls);
        }

        // Search status
        if (ballController.getState() == SimpleBallAlignmentController.AlignmentState.SEARCHING) {
            telemetry.addLine();
            telemetry.addLine("──── SEARCHING ────");
            telemetry.addData("Direction", "CCW");
            telemetry.addData("Speed", SimpleBallAlignmentController.BallAlignSearch.SEARCH_SPEED);
        }

        telemetry.addLine("╚═══════════════════════════╝");
    }
}