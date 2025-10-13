package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.BallPursuitController;

/**
 * Ball Pursuit Test - Tests dynamic ball tracking and following
 * Press START to begin ball pursuit
 */
@TeleOp(name = "Ball Pursuit Test", group = "Test")
public class BallPursuitTest extends LinearOpMode {

    private BallPursuitController ballPursuit;
    private ElapsedTime runtime = new ElapsedTime();

    // Performance tracking
    private int pursuitStartedCount = 0;
    private int ballReachedCount = 0;
    private int ballLostCount = 0;
    private ElapsedTime pursuitTimer = new ElapsedTime();
    private boolean pursuitActive = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Ball Pursuit...");
        telemetry.update();

        // Initialize pursuit controller
        try {
            ballPursuit = new BallPursuitController(this);
            telemetry.addLine("✓ Ball Pursuit Initialized");
            telemetry.addLine("✓ Controllers Ready");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Initialization failed: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // Display startup info
        telemetry.addLine();
        telemetry.addLine("╔═══════ BALL PURSUIT ═══════╗");
        telemetry.addLine("║ Dynamic ball tracking     ║");
        telemetry.addLine("║ Align + Path following    ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("──── KEY FEATURES ────");
        telemetry.addData("Aligns", "PID heading control");
        telemetry.addData("Pursues", "Pure pursuit pathing");
        telemetry.addData("Updates", "Dynamic path recalculation");
        telemetry.addData("Stops", "At ball or when lost");
        telemetry.addLine();
        telemetry.addLine("Press START to begin pursuit");
        telemetry.update();

        waitForStart();
        runtime.reset();
        pursuitTimer.reset();

        // Main loop
        while (opModeIsActive()) {
            // Main pursuit logic
            ballPursuit.updatePursuit();

            // Track performance statistics
            trackPerformance();

            // Display comprehensive telemetry
            displayTelemetry();
            telemetry.update();

            // Standard FTC loop timing
            sleep(10);
        }

        // Cleanup
        ballPursuit.stopPursuit();
    }

    private void trackPerformance() {
        BallPursuitController.PursuitState state = ballPursuit.getState();

        // Track pursuit starts
        if (state != BallPursuitController.PursuitState.STOPPED && !pursuitActive) {
            pursuitStartedCount++;
            pursuitActive = true;
            pursuitTimer.reset();
        } else if (state == BallPursuitController.PursuitState.STOPPED && pursuitActive) {
            pursuitActive = false;
        }

        // Track reaching ball
        if (state == BallPursuitController.PursuitState.AT_BALL) {
            ballReachedCount++;
        }

        // Track ball loss
        if (state == BallPursuitController.PursuitState.BALL_LOST) {
            ballLostCount++;
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("╔═══════ BALL PURSUIT ═══════╗");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Pursuit Active", ballPursuit.isPursuing());
        telemetry.addLine();

        // Pursuit status
        ballPursuit.displayTelemetry();

        // PID Configuration
        telemetry.addLine();
        telemetry.addLine("──── PURSUIT CONFIG ────");
        telemetry.addData("Approach Dist", "%.1f in", BallPursuitController.BallPursuitConfig.APPROACH_DISTANCE);
        telemetry.addData("Stop Dist", "%.1f in", BallPursuitController.BallPursuitConfig.STOP_DISTANCE);
        telemetry.addData("Max Speed", "%.1f", BallPursuitController.BallPursuitConfig.MAX_SPEED);
        telemetry.addData("Min Speed", "%.1f", BallPursuitController.BallPursuitConfig.MIN_SPEED);
        telemetry.addData("Update Interval", "%.1f sec", BallPursuitController.BallPursuitConfig.PATH_UPDATE_INTERVAL);
        telemetry.addData("Lost Timeout", "%.1f sec", BallPursuitController.BallPursuitConfig.BALL_LOST_TIMEOUT);
        telemetry.addData("Heading Weight", "%.1f", BallPursuitController.BallPursuitConfig.HEADING_CORRECTION_WEIGHT);

        // Statistics
        if (pursuitStartedCount > 0) {
            telemetry.addLine();
            telemetry.addLine("──── STATISTICS ────");
            telemetry.addData("Pursuits Started", pursuitStartedCount);
            telemetry.addData("Balls Reached", ballReachedCount);
            telemetry.addData("Balls Lost", ballLostCount);

            if (pursuitActive) {
                telemetry.addData("Current Time", "%.1f sec", pursuitTimer.seconds());
            }

            double successRate = ballReachedCount / (double) pursuitStartedCount * 100.0;
            telemetry.addData("Success Rate", "%.1f%%", successRate);
        }

        telemetry.addLine("╚═══════════════════════════╝");
    }
}