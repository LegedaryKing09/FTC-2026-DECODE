package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

/**
 * Comprehensive Road Runner Tuning Test Suite
 *
 * This OpMode contains multiple tests to help you tune your Road Runner parameters.
 * Use FTC Dashboard (192.168.43.1:8080/dash) to monitor telemetry in real-time.
 *
 * Tests included:
 * 1. Max Velocity Test - Determine maxWheelVelTick
 * 2. Acceleration Test - Tune maxProfileAccel and minProfileAccel
 * 3. Turn Test - Tune turnGain and turnVelGain
 * 4. Straight Line Test - Verify velocity and acceleration
 * 5. Trajectory Test - Test Ramsete controller (ramseteZeta, ramseteBBar)
 *
 * Instructions:
 * 1. Connect to FTC Dashboard
 * 2. Run this OpMode
 * 3. Select which test to run using gamepad
 * 4. Observe telemetry and adjust PARAMS values in AutoTankDrive
 * 5. Re-run tests until behavior is satisfactory
 */
@Autonomous(name = "Road Runner Tuning Tests", group = "Tuning")
public class RoadRunnerTuningTest extends LinearOpMode {

    private AutoTankDrive drive;
    private FtcDashboard dashboard;
    private ElapsedTime runtime;

    // Test selection
    private enum TestMode {
        IDLE,
        MAX_VELOCITY,
        ACCELERATION,
        TURN_TEST,
        STRAIGHT_LINE,
        TRAJECTORY_TEST
    }

    private TestMode currentTest = TestMode.IDLE;

    @Override
    public void runOpMode() {
        // Initialize dashboard for real-time graphing
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize drive
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new AutoTankDrive(hardwareMap, startPose);
        runtime = new ElapsedTime();

        telemetry.addLine("=== ROAD RUNNER TUNING TESTS ===");
        telemetry.addLine();
        telemetry.addLine("Connect to FTC Dashboard:");
        telemetry.addLine("192.168.43.1:8080/dash");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A - Max Velocity Test");
        telemetry.addLine("B - Acceleration Test");
        telemetry.addLine("X - Turn Test");
        telemetry.addLine("Y - Straight Line Test");
        telemetry.addLine("DPAD_UP - Trajectory Test");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Test selection with gamepad
            if (gamepad1.a) {
                runMaxVelocityTest();
            } else if (gamepad1.b) {
                runAccelerationTest();
            } else if (gamepad1.x) {
                runTurnTest();
            } else if (gamepad1.y) {
                runStraightLineTest();
            } else if (gamepad1.dpad_up) {
                runTrajectoryTest();
            }

            // Display menu when idle
            if (currentTest == TestMode.IDLE) {
                telemetry.addLine("=== SELECT A TEST ===");
                telemetry.addLine();
                telemetry.addLine("A - Max Velocity Test");
                telemetry.addLine("   Find maxWheelVelTick");
                telemetry.addLine();
                telemetry.addLine("B - Acceleration Test");
                telemetry.addLine("   Tune maxProfileAccel/minProfileAccel");
                telemetry.addLine();
                telemetry.addLine("X - Turn Test");
                telemetry.addLine("   Tune turnGain/turnVelGain");
                telemetry.addLine();
                telemetry.addLine("Y - Straight Line Test");
                telemetry.addLine("   Verify velocity constraints");
                telemetry.addLine();
                telemetry.addLine("DPAD_UP - Trajectory Test");
                telemetry.addLine("   Test Ramsete (zeta/bBar)");
                telemetry.update();
            }

            sleep(100);
        }
    }

    /**
     * TEST 1: MAX VELOCITY TEST
     *
     * Purpose: Determine the maximum safe wheel velocity (maxWheelVelTick)
     *
     * Instructions:
     * 1. This test will run motors at full power for 3 seconds
     * 2. Watch the "Peak Velocity" value in telemetry
     * 3. Set maxWheelVelTick to ~80-90% of the peak value
     * 4. Example: If peak is 7200, set maxWheelVelTick = 6500
     */
    private void runMaxVelocityTest() {
        currentTest = TestMode.MAX_VELOCITY;

        telemetry.clearAll();
        telemetry.addLine("=== MAX VELOCITY TEST ===");
        telemetry.addLine();
        telemetry.addLine("Running motors at full power...");
        telemetry.addLine("Drive will move forward!");
        telemetry.addLine();
        telemetry.addLine("Watch 'Peak Velocity' value");
        telemetry.update();

        // Set motors to use encoders for velocity reading
        for (DcMotor motor : drive.leftMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotor motor : drive.rightMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double maxVelocity = 0;
        runtime.reset();

        // Run at full power for 3 seconds
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            // Full power forward
            for (DcMotor motor : drive.leftMotors) {
                motor.setPower(1.0);
            }
            for (DcMotor motor : drive.rightMotors) {
                motor.setPower(1.0);
            }

            // Read velocities (using Pinpoint odometry)
            drive.updatePoseEstimate();
            double currentVel = Math.abs(drive.pinpointLocalizer.driver.getEncoderY())
                    / AutoTankDrive.PARAMS.odoInPerTick;

            if (currentVel > maxVelocity) {
                maxVelocity = currentVel;
            }

            telemetry.addData("Time", "%.1f / 3.0 sec", runtime.seconds());
            telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVel);
            telemetry.addData("Peak Velocity", "%.0f ticks/sec", maxVelocity);
            telemetry.addLine();
            telemetry.addData("Recommended maxWheelVelTick", "%.0f", maxVelocity * 0.85);
            telemetry.addLine();
            telemetry.update();
        }

        // Stop motors
        stopAllMotors();

        telemetry.addLine();
        telemetry.addLine("TEST COMPLETE!");
        telemetry.addLine();
        telemetry.addData("Peak Velocity", "%.0f ticks/sec", maxVelocity);
        telemetry.addData("Recommended maxWheelVelTick", "%.0f (85%% of peak)", maxVelocity * 0.85);
        telemetry.addLine();
        telemetry.addLine("Update AutoTankDrive.PARAMS.maxWheelVelTick");
        telemetry.addLine("Press B for next test");
        telemetry.update();

        sleep(5000);
        currentTest = TestMode.IDLE;
    }

    /**
     * TEST 2: ACCELERATION TEST
     *
     * Purpose: Tune maxProfileAccel and minProfileAccel
     *
     * Instructions:
     * 1. Robot will accelerate and decelerate multiple times
     * 2. Watch for wheel slip during acceleration
     * 3. Watch for skidding during braking
     * 4. Adjust maxProfileAccel if wheels slip on acceleration
     * 5. Adjust minProfileAccel if wheels skid on braking
     *
     * Current values should be symmetric (e.g., 60 and -60)
     */
    private void runAccelerationTest() {
        currentTest = TestMode.ACCELERATION;

        telemetry.clearAll();
        telemetry.addLine("=== ACCELERATION TEST ===");
        telemetry.addLine();
        telemetry.addLine("Robot will drive forward");
        telemetry.addLine("Watch for wheel slip!");
        telemetry.update();

        sleep(2000);

        // Reset pose
        drive.pinpointLocalizer.setPose(new Pose2d(0, 0, 0));

        // Build a trajectory with acceleration/deceleration
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(48) // Drive 48 inches forward
                        .build()
        );

        telemetry.addLine();
        telemetry.addLine("TEST COMPLETE!");
        telemetry.addLine();
        telemetry.addLine("Observations:");
        telemetry.addLine("- Did wheels slip during acceleration?");
        telemetry.addLine("  → Decrease maxProfileAccel");
        telemetry.addLine("- Did wheels skid during braking?");
        telemetry.addLine("  → Decrease |minProfileAccel|");
        telemetry.addLine("- Too slow/sluggish?");
        telemetry.addLine("  → Increase both values");
        telemetry.addLine();
        telemetry.addData("Current maxProfileAccel", AutoTankDrive.PARAMS.maxProfileAccel);
        telemetry.addData("Current minProfileAccel", AutoTankDrive.PARAMS.minProfileAccel);
        telemetry.addLine();
        telemetry.addLine("Recommended: Use symmetric values");
        telemetry.addLine("Example: maxProfileAccel=60, minProfileAccel=-60");
        telemetry.update();

        sleep(8000);
        currentTest = TestMode.IDLE;
    }

    /**
     * TEST 3: TURN TEST
     *
     * Purpose: Tune turnGain and turnVelGain
     *
     * Instructions:
     * 1. Robot will perform 90° and 180° turns
     * 2. Watch heading error in telemetry
     * 3. If robot undershoots → increase turnGain
     * 4. If robot oscillates → decrease turnGain or increase turnVelGain
     * 5. Start with turnGain=2.0, turnVelGain=0.3
     */
    private void runTurnTest() {
        currentTest = TestMode.TURN_TEST;

        telemetry.clearAll();
        telemetry.addLine("=== TURN TEST ===");
        telemetry.addLine();
        telemetry.addLine("Robot will turn 90° and 180°");
        telemetry.update();

        sleep(2000);

        // Reset pose
        drive.pinpointLocalizer.setPose(new Pose2d(0, 0, 0));

        // Perform turns and measure accuracy
        telemetry.addLine("Turning 90°...");
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .turn(Math.toRadians(90))
                        .build()
        );

        drive.updatePoseEstimate();
        double heading90 = Math.toDegrees(drive.pinpointLocalizer.getPose().heading.toDouble());
        double error90 = 90 - heading90;

        sleep(1000);

        telemetry.addLine("Turning 180°...");
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pinpointLocalizer.getPose())
                        .turn(Math.toRadians(180))
                        .build()
        );

        drive.updatePoseEstimate();
        double heading270 = Math.toDegrees(drive.pinpointLocalizer.getPose().heading.toDouble());
        double error270 = 270 - heading270;

        telemetry.addLine();
        telemetry.addLine("TEST COMPLETE!");
        telemetry.addLine();
        telemetry.addData("90° Turn Error", "%.2f degrees", error90);
        telemetry.addData("270° Turn Error", "%.2f degrees", error270);
        telemetry.addLine();
        telemetry.addLine("Analysis:");
        if (Math.abs(error90) > 5 || Math.abs(error270) > 5) {
            telemetry.addLine("→ Large error: Increase turnGain");
        }
        if (Math.abs(error90) < 2 && Math.abs(error270) < 2) {
            telemetry.addLine("→ Good accuracy!");
        }
        telemetry.addLine();
        telemetry.addData("Current turnGain", AutoTankDrive.PARAMS.turnGain);
        telemetry.addData("Current turnVelGain", AutoTankDrive.PARAMS.turnVelGain);
        telemetry.addLine();
        telemetry.addLine("Recommended starting values:");
        telemetry.addLine("turnGain = 2.0");
        telemetry.addLine("turnVelGain = 0.3");
        telemetry.update();

        sleep(8000);
        currentTest = TestMode.IDLE;
    }

    /**
     * TEST 4: STRAIGHT LINE TEST
     *
     * Purpose: Verify velocity constraints work correctly
     *
     * Instructions:
     * 1. Robot drives straight for 60 inches
     * 2. Watch velocity graphs on FTC Dashboard
     * 3. Velocity should reach near maxWheelVel
     * 4. If too cautious → increase maxWheelVel
     * 5. If brownout/slip → decrease maxWheelVel
     */
    private void runStraightLineTest() {
        currentTest = TestMode.STRAIGHT_LINE;

        telemetry.clearAll();
        telemetry.addLine("=== STRAIGHT LINE TEST ===");
        telemetry.addLine();
        telemetry.addLine("Robot will drive 60 inches forward");
        telemetry.addLine("Watch velocity on Dashboard!");
        telemetry.update();

        sleep(2000);

        drive.pinpointLocalizer.setPose(new Pose2d(0, 0, 0));

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(60)
                        .build()
        );

        telemetry.addLine();
        telemetry.addLine("TEST COMPLETE!");
        telemetry.addLine();
        telemetry.addLine("Check Dashboard graphs:");
        telemetry.addLine("- Did velocity reach maxWheelVel?");
        telemetry.addLine("- Any brownouts or wheel slip?");
        telemetry.addLine("- Smooth acceleration curve?");
        telemetry.addLine();
        telemetry.addData("maxWheelVel", AutoTankDrive.PARAMS.maxWheelVel + " in/s");
        telemetry.update();

        sleep(5000);
        currentTest = TestMode.IDLE;
    }

    /**
     * TEST 5: TRAJECTORY TEST (S-CURVE)
     *
     * Purpose: Tune Ramsete controller (ramseteZeta, ramseteBBar)
     *
     * Instructions:
     * 1. Robot follows an S-curve trajectory
     * 2. Watch path tracking on Dashboard field view
     * 3. Watch xError, yError, headingError in telemetry
     * 4. If oscillating → increase ramseteZeta (try 0.8-0.9)
     * 5. If lagging/cutting corners → increase ramseteBBar (try 2.5-3.0)
     *
     * Recommended: ramseteZeta=0.7, ramseteBBar=2.0
     */
    private void runTrajectoryTest() {
        currentTest = TestMode.TRAJECTORY_TEST;

        telemetry.clearAll();
        telemetry.addLine("=== TRAJECTORY TEST ===");
        telemetry.addLine();
        telemetry.addLine("Robot will follow S-curve");
        telemetry.addLine("Watch Dashboard field view!");
        telemetry.update();

        sleep(2000);

        drive.pinpointLocalizer.setPose(new Pose2d(0, 0, 0));

        // Complex S-curve trajectory
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(30, 30), Math.toRadians(0))
                        .splineTo(new Vector2d(60, 0), Math.toRadians(0))
                        .build()
        );

        telemetry.addLine();
        telemetry.addLine("TEST COMPLETE!");
        telemetry.addLine();
        telemetry.addLine("Analysis (check Dashboard):");
        telemetry.addLine("- Blue line = actual path");
        telemetry.addLine("- Green line = target path");
        telemetry.addLine();
        telemetry.addLine("Common issues:");
        telemetry.addLine("→ Oscillating around path:");
        telemetry.addLine("  Increase ramseteZeta to 0.8-0.9");
        telemetry.addLine("→ Cutting inside curves:");
        telemetry.addLine("  Increase ramseteBBar to 2.5-3.0");
        telemetry.addLine("→ Lagging behind:");
        telemetry.addLine("  Increase ramseteBBar");
        telemetry.addLine();
        telemetry.addData("Current ramseteZeta", AutoTankDrive.PARAMS.ramseteZeta);
        telemetry.addData("Current ramseteBBar", AutoTankDrive.PARAMS.ramseteBBar);
        telemetry.update();

        sleep(10000);
        currentTest = TestMode.IDLE;
    }

    private void stopAllMotors() {
        for (DcMotor motor : drive.leftMotors) {
            motor.setPower(0);
        }
        for (DcMotor motor : drive.rightMotors) {
            motor.setPower(0);
        }
    }
}