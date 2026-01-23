package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Axon Mini MK2 Continuous Test", group = "Test")
public class AxonServoContinuousTest extends LinearOpMode {

    private CRServo axonServo;
    private AnalogInput axonAnalog;

    // Button state tracking
    private boolean isPressingA = false;
    private boolean isPressingB = false;
    private boolean isPressingX = false;

    // Servo power constants for continuous mode
    private static final double STOP_POWER = 0.0;
    private static final double CLOCKWISE_POWER = 0.5;
    private static final double COUNTER_CLOCKWISE_POWER = -0.5;

    // Precision movement settings
    private static final double MOVEMENT_INCREMENT_DEGREES = 10.0;  // 10 degrees of actual rotation
    private static final double POSITION_TOLERANCE = 0.003;   // ~1 degrees in position units (1/360)
    private static final long MOVEMENT_TIMEOUT = 3000;

    // Movement state
    private boolean isMoving = false;
    private double targetPosition = 0.0;  // Changed to position (0.0-1.0)
    private long movementStartTime = 0;

    @Override
    public void runOpMode() {
        axonServo = hardwareMap.get(CRServo.class, "axon_servo");
        axonAnalog = hardwareMap.get(AnalogInput.class, "axon_analog");

        telemetry.addLine("=== Axon Mini MK2 Precision Control ===");
        telemetry.addLine("Waiting for start...");
        telemetry.addLine();
        telemetry.addLine("A: Turn 10° Clockwise");
        telemetry.addLine("B: Turn 10° Counter-Clockwise");
        telemetry.addLine("X: Emergency Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read current position from analog feedback
            double analogVoltage = axonAnalog.getVoltage();
            double currentPosition = analogVoltage / 3.3;  // 0.0-1.0
            double currentAngle = currentPosition * 360.0;  // 0-360 degrees

            // A button: Turn 10 degrees clockwise
            if (gamepad1.a && !isPressingA && !isMoving) {
                isPressingA = true;
                // Convert 10 degrees to position increment
                double positionIncrement = MOVEMENT_INCREMENT_DEGREES / 360.0;
                startMovement(currentPosition, positionIncrement);
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // B button: Turn 10 degrees counter-clockwise
            if (gamepad1.b && !isPressingB && !isMoving) {
                isPressingB = true;
                // Convert 10 degrees to position increment (negative)
                double positionIncrement = -MOVEMENT_INCREMENT_DEGREES / 360.0;
                startMovement(currentPosition, positionIncrement);
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            // X button: Emergency stop
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                axonServo.setPower(STOP_POWER);
                isMoving = false;
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            // Handle precision movement
            if (isMoving) {
                double error = getPositionError(currentPosition, targetPosition);
                double absError = Math.abs(error);

                // Check if we've reached the target
                if (absError <= POSITION_TOLERANCE) {
                    axonServo.setPower(STOP_POWER);
                    isMoving = false;
                }
                // Check for timeout
                else if (System.currentTimeMillis() - movementStartTime > MOVEMENT_TIMEOUT) {
                    axonServo.setPower(STOP_POWER);
                    isMoving = false;
                    telemetry.addLine("⚠️ TIMEOUT - Movement took too long");
                }
                // Continue moving toward target
                else {
                    // Determine direction based on error sign
                    if (error > 0) {
                        axonServo.setPower(CLOCKWISE_POWER);
                    } else {
                        axonServo.setPower(COUNTER_CLOCKWISE_POWER);
                    }
                }
            }

            // Determine current status
            String servoStatus = isMoving ? "MOVING TO TARGET" : "IDLE";

            // Display telemetry
            telemetry.addLine("=== AXON MINI MK2 STATUS ===");
            telemetry.addData("Status", servoStatus);
            telemetry.addData("Servo Power", "%.2f", axonServo.getPower());
            telemetry.addLine();

            telemetry.addLine("=== POSITION INFO ===");
            telemetry.addData("Current Position", "%.3f", currentPosition);
            telemetry.addData("Current Angle", "%.1f°", currentAngle);

            if (isMoving) {
                double error = getPositionError(currentPosition, targetPosition);
                double targetAngle = targetPosition * 360.0;
                telemetry.addData("Target Position", "%.3f", targetPosition);
                telemetry.addData("Target Angle", "%.1f°", targetAngle);
                telemetry.addData("Error (position)", "%.4f", error);
                telemetry.addData("Error (degrees)", "%.1f°", error * 360.0);
                telemetry.addData("Time Elapsed", "%d ms", System.currentTimeMillis() - movementStartTime);
            }
            telemetry.addLine();

            telemetry.addLine("=== ANALOG FEEDBACK ===");
            telemetry.addData("Raw Voltage", "%.3f V", analogVoltage);
            telemetry.addLine();

            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A: Turn 10° Clockwise");
            telemetry.addLine("B: Turn 10° Counter-Clockwise");
            telemetry.addLine("X: Emergency Stop");

            telemetry.update();
        }

        // Safety: Stop servo when OpMode ends
        axonServo.setPower(STOP_POWER);
    }

    /**
     * Start a precision movement to a new target position
     * @param currentPosition Current position (0.0-1.0)
     * @param increment Amount to move in position units
     */
    private void startMovement(double currentPosition, double increment) {
        targetPosition = normalizePosition(currentPosition + increment);
        isMoving = true;
        movementStartTime = System.currentTimeMillis();
    }

    /**
     * Normalize a position to be within 0.0-1.0
     * @param position Position value
     * @return Normalized position (0.0-1.0)
     */
    private double normalizePosition(double position) {
        position = position % 1.0;
        if (position < 0) {
            position += 1.0;
        }
        return position;
    }

    /**
     * Calculate the shortest position error between current and target
     * Handles wraparound at 0.0/1.0
     * @param current Current position (0.0-1.0)
     * @param target Target position (0.0-1.0)
     * @return Error in position units (positive = need to rotate clockwise, negative = counter-clockwise)
     */
    private double getPositionError(double current, double target) {
        double error = target - current;

        // Handle wraparound: choose the shortest path
        if (error > 0.5) {
            error -= 1.0;
        } else if (error < -0.5) {
            error += 1.0;
        }

        return error;
    }
}