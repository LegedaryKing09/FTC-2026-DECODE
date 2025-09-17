package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.ServoController;

@TeleOp(name = "Spindex Test", group = "Test")
public class SpindexTest extends LinearOpMode {

    private ServoController axonController;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime modeTimer = new ElapsedTime();

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private TestMode currentTestMode = TestMode.MANUAL;
    private AutoTestState autoTestState = AutoTestState.STOPPED;
    private double autoTestStartTime = 0;

    private enum TestMode {
        MANUAL, AUTO_SEQUENCE
    }

    private enum AutoTestState {
        STOPPED, FORWARD_FULL, FORWARD_HALF, FORWARD_QUARTER, FORWARD_SLOW,
        REVERSE_FULL, REVERSE_HALF, REVERSE_QUARTER, REVERSE_SLOW, COMPLETE
    }

    @Override
    public void runOpMode() {
        // Initialize the controller
        axonController = new ServoController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("DPAD UP: Forward Full");
        telemetry.addLine("DPAD DOWN: Reverse Full");
        telemetry.addLine("DPAD LEFT: Forward Half");
        telemetry.addLine("DPAD RIGHT: Reverse Half");
        telemetry.addLine("Y: Forward Quarter");
        telemetry.addLine("A: Reverse Quarter");
        telemetry.addLine("X: Forward Slow");
        telemetry.addLine("B: Reverse Slow");
        telemetry.addLine("LEFT BUMPER: Stop");
        telemetry.addLine("RIGHT BUMPER: Toggle Auto Test");
        telemetry.addLine("LEFT/RIGHT TRIGGERS: Manual Power Control");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            handleInput();

            if (currentTestMode == TestMode.AUTO_SEQUENCE) {
                runAutoTest();
            }

            updateTelemetry();

            // Store previous button states
            storePreviousButtonStates();
        }

        // Ensure servo stops when OpMode ends
        axonController.stop();
    }

    private void handleInput() {
        // Manual control buttons (only work in manual mode)
        if (currentTestMode == TestMode.MANUAL) {
            // DPAD controls
            if (gamepad1.dpad_up && !lastDpadUp) {
                axonController.forwardFull();
                telemetry.addData("Command", "Forward Full");
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                axonController.reverseFull();
                telemetry.addData("Command", "Reverse Full");
            }
            if (gamepad1.dpad_left && !lastDpadLeft) {
                axonController.forwardHalf();
                telemetry.addData("Command", "Forward Half");
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                axonController.reverseHalf();
                telemetry.addData("Command", "Reverse Half");
            }

            // Face button controls
            if (gamepad1.y && !lastY) {
                axonController.forwardQuarter();
                telemetry.addData("Command", "Forward Quarter");
            }
            if (gamepad1.a && !lastA) {
                axonController.reverseQuarter();
                telemetry.addData("Command", "Reverse Quarter");
            }
            if (gamepad1.x && !lastX) {
                axonController.forwardSlow();
                telemetry.addData("Command", "Forward Slow");
            }
            if (gamepad1.b && !lastB) {
                axonController.reverseSlow();
                telemetry.addData("Command", "Reverse Slow");
            }

            // Trigger controls for variable power
            if (gamepad1.right_trigger > 0.1) {
                double power = gamepad1.right_trigger;
                axonController.setPower(power);
                telemetry.addData("Command", "Manual Forward: %.2f", power);
            } else if (gamepad1.left_trigger > 0.1) {
                double power = -gamepad1.left_trigger;
                axonController.setPower(power);
                telemetry.addData("Command", "Manual Reverse: %.2f", power);
            }
        }

        // Stop button (works in both modes)
        if (gamepad1.left_bumper && !lastLeftBumper) {
            axonController.stop();
            if (currentTestMode == TestMode.AUTO_SEQUENCE) {
                autoTestState = AutoTestState.STOPPED;
            }
            telemetry.addData("Command", "Stop");
        }

        // Mode toggle
        if (gamepad1.right_bumper && !lastRightBumper) {
            if (currentTestMode == TestMode.MANUAL) {
                currentTestMode = TestMode.AUTO_SEQUENCE;
                autoTestState = AutoTestState.FORWARD_FULL;
                autoTestStartTime = runtime.seconds();
                modeTimer.reset();
                telemetry.addData("Mode", "Switched to Auto Test");
            } else {
                currentTestMode = TestMode.MANUAL;
                autoTestState = AutoTestState.STOPPED;
                axonController.stop();
                telemetry.addData("Mode", "Switched to Manual");
            }
        }
    }

    private void runAutoTest() {
        double currentTime = runtime.seconds();
        double stateTime = currentTime - autoTestStartTime;

        // Each state runs for 2 seconds
        if (stateTime >= 2.0) {
            autoTestStartTime = currentTime;

            switch (autoTestState) {
                case FORWARD_FULL:
                    axonController.forwardHalf();
                    autoTestState = AutoTestState.FORWARD_HALF;
                    break;
                case FORWARD_HALF:
                    axonController.forwardQuarter();
                    autoTestState = AutoTestState.FORWARD_QUARTER;
                    break;
                case FORWARD_QUARTER:
                    axonController.forwardSlow();
                    autoTestState = AutoTestState.FORWARD_SLOW;
                    break;
                case FORWARD_SLOW:
                    axonController.reverseFull();
                    autoTestState = AutoTestState.REVERSE_FULL;
                    break;
                case REVERSE_FULL:
                    axonController.reverseHalf();
                    autoTestState = AutoTestState.REVERSE_HALF;
                    break;
                case REVERSE_HALF:
                    axonController.reverseQuarter();
                    autoTestState = AutoTestState.REVERSE_QUARTER;
                    break;
                case REVERSE_QUARTER:
                    axonController.reverseSlow();
                    autoTestState = AutoTestState.REVERSE_SLOW;
                    break;
                case REVERSE_SLOW:
                    axonController.stop();
                    autoTestState = AutoTestState.COMPLETE;
                    break;
                case COMPLETE:
                    // Test complete, switch back to manual mode
                    currentTestMode = TestMode.MANUAL;
                    autoTestState = AutoTestState.STOPPED;
                    break;
                case STOPPED:
                    axonController.forwardFull();
                    autoTestState = AutoTestState.FORWARD_FULL;
                    break;
            }
        } else if (autoTestState == AutoTestState.STOPPED) {
            // Start the test sequence
            axonController.forwardFull();
            autoTestState = AutoTestState.FORWARD_FULL;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
        telemetry.addData("Test Mode", currentTestMode);

        if (currentTestMode == TestMode.AUTO_SEQUENCE) {
            telemetry.addData("Auto Test State", autoTestState);
            double stateTime = runtime.seconds() - autoTestStartTime;
            telemetry.addData("State Time", "%.1f / 2.0 seconds", stateTime);
        }

        telemetry.addLine();
        telemetry.addData("Servo Power", "%.3f", axonController.getPower());
        telemetry.addData("Servo Mode", axonController.getMode());
        telemetry.addData("Is Moving Forward", axonController.isMovingForward());
        telemetry.addData("Is Moving Reverse", axonController.isMovingReverse());
        telemetry.addData("Is Stopped", axonController.isStopped());

        telemetry.addLine();
        telemetry.addLine("=== POWER CONSTANTS ===");
        telemetry.addData("Full Power", ServoController.AXON_MINI_FULL_POWER);
        telemetry.addData("Half Power", ServoController.AXON_MINI_HALF_POWER);
        telemetry.addData("Quarter Power", ServoController.AXON_MINI_QUARTER_POWER);
        telemetry.addData("Slow Power", ServoController.AXON_MINI_SLOW_POWER);

        telemetry.update();
    }

    private void storePreviousButtonStates() {
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }
}