package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@Config
@TeleOp(name = "Six Motor Direction Test")
public class AllDirection extends LinearOpMode {

    // Motor configuration names - CHANGE THESE TO MATCH YOUR HARDWARE CONFIG
    public static String L1_NAME = "lf";  // Left Front
    public static String L2_NAME = "lm";  // Left Middle
    public static String L3_NAME = "lb";  // Left Back
    public static String R1_NAME = "rf";  // Right Front
    public static String R2_NAME = "rm";  // Right Middle
    public static String R3_NAME = "rb";  // Right Back

    // Test power level
    public static double TEST_POWER = 1;

    // Motor references
    private DcMotor motor1Left, motor2Left, motor3Left;
    private DcMotor motor1Right, motor2Right, motor3Right;

    // Current combination (0 to 63 = 2^6 combinations)
    private int currentCombination = 0;
    private final int TOTAL_COMBINATIONS = 64;

    @Override
    public void runOpMode() {
        // Initialize motors
        motor1Left = hardwareMap.get(DcMotor.class, L1_NAME);
        motor2Left = hardwareMap.get(DcMotor.class, L2_NAME);
        motor3Left = hardwareMap.get(DcMotor.class, L3_NAME);

        motor1Right = hardwareMap.get(DcMotor.class, R1_NAME);
        motor2Right = hardwareMap.get(DcMotor.class, R2_NAME);
        motor3Right = hardwareMap.get(DcMotor.class, R3_NAME);

        // Set brake mode
        motor1Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders
        motor1Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean isPressingDpadUp = false;
        boolean isPressingDpadDown = false;
        boolean isPressingA = false;
        boolean isPressingB = false;
        boolean isPressingX = false;
        boolean isPressingY = false;

        telemetry.addLine("=== SIX MOTOR DIRECTION TEST ===");
        telemetry.addLine();
        telemetry.addLine("This will test all 64 possible");
        telemetry.addLine("direction combinations for 6 motors.");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("DPAD UP: Next combination");
        telemetry.addLine("DPAD DOWN: Previous combination");
        telemetry.addLine("HOLD A: Test drive FORWARD");
        telemetry.addLine("HOLD B: Test turn RIGHT");
        telemetry.addLine("HOLD X: Test turn LEFT");
        telemetry.addLine("Y: STOP motors");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // Apply initial combination
        applyCombination(currentCombination);

        while (opModeIsActive()) {
            // Navigation - Next combination
            if (gamepad1.dpad_up && !isPressingDpadUp) {
                isPressingDpadUp = true;
                stopAllMotors();
                currentCombination = (currentCombination + 1) % TOTAL_COMBINATIONS;
                applyCombination(currentCombination);
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            // Navigation - Previous combination
            if (gamepad1.dpad_down && !isPressingDpadDown) {
                isPressingDpadDown = true;
                stopAllMotors();
                currentCombination = (currentCombination - 1 + TOTAL_COMBINATIONS) % TOTAL_COMBINATIONS;
                applyCombination(currentCombination);
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }

            // A - Test drive forward
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                testDriveForward();
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
                stopAllMotors();
            }

            // B - Test turn right
            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                testTurnRight();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
                stopAllMotors();
            }

            // X - Test turn left
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                testTurnLeft();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
                stopAllMotors();
            }

            // Y - Stop all motors
            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                stopAllMotors();
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            // Display telemetry
            displayTelemetry();
            telemetry.update();
        }
    }

    private void applyCombination(int combination) {
        // Each bit represents one motor's direction
        // Bit 0 = L1, Bit 1 = L2, Bit 2 = L3, Bit 3 = R1, Bit 4 = R2, Bit 5 = R3
        // 0 = FORWARD, 1 = REVERSE

        motor1Left.setDirection((combination & (1 << 0)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2Left.setDirection((combination & (1 << 1)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor3Left.setDirection((combination & (1 << 2)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        motor1Right.setDirection((combination & (1 << 3)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2Right.setDirection((combination & (1 << 4)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor3Right.setDirection((combination & (1 << 5)) != 0 ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void testDriveForward() {
        // Set all motors to same power for forward drive
        motor1Left.setPower(TEST_POWER);
        motor2Left.setPower(TEST_POWER);
        motor3Left.setPower(TEST_POWER);

        motor1Right.setPower(TEST_POWER);
        motor2Right.setPower(TEST_POWER);
        motor3Right.setPower(TEST_POWER);
    }

    private void testTurnRight() {
        // Left side forward, right side backward for right turn
        motor1Left.setPower(TEST_POWER);
        motor2Left.setPower(TEST_POWER);
        motor3Left.setPower(TEST_POWER);

        motor1Right.setPower(-TEST_POWER);
        motor2Right.setPower(-TEST_POWER);
        motor3Right.setPower(-TEST_POWER);
    }

    private void testTurnLeft() {
        // Left side backward, right side forward for left turn
        motor1Left.setPower(-TEST_POWER);
        motor2Left.setPower(-TEST_POWER);
        motor3Left.setPower(-TEST_POWER);

        motor1Right.setPower(TEST_POWER);
        motor2Right.setPower(TEST_POWER);
        motor3Right.setPower(TEST_POWER);
    }

    private void stopAllMotors() {
        motor1Left.setPower(0);
        motor2Left.setPower(0);
        motor3Left.setPower(0);

        motor1Right.setPower(0);
        motor2Right.setPower(0);
        motor3Right.setPower(0);
    }

    private String getDirectionString(DcMotor motor) {
        return motor.getDirection() == DcMotorSimple.Direction.FORWARD ? "FWD" : "REV";
    }

    private String getBinaryRepresentation(int combination) {
        // Returns binary string like "001101" for visualization
        // Format: R3 R2 R1 L3 L2 L1
        StringBuilder binary = new StringBuilder();
        for (int i = 5; i >= 0; i--) {
            binary.append((combination & (1 << i)) != 0 ? "1" : "0");
        }
        return binary.toString();
    }

    private void displayTelemetry() {
        telemetry.addLine("=== SIX MOTOR DIRECTION TEST ===");
        telemetry.addLine();

        telemetry.addData("Combination", String.format("%d / %d (Binary: %s)",
                currentCombination + 1, TOTAL_COMBINATIONS,
                getBinaryRepresentation(currentCombination)));
        telemetry.addLine();

        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  DPAD UP: Next combo");
        telemetry.addLine("  DPAD DOWN: Previous combo");
        telemetry.addLine("  HOLD A: Test FORWARD");
        telemetry.addLine("  HOLD B: Test turn RIGHT");
        telemetry.addLine("  HOLD X: Test turn LEFT");
        telemetry.addLine("  Y: Stop motors");
        telemetry.addLine();

        telemetry.addLine("--- LEFT SIDE DIRECTIONS ---");
        telemetry.addData("L1 (Left Front)", getDirectionString(motor1Left));
        telemetry.addData("L2 (Left Middle)", getDirectionString(motor2Left));
        telemetry.addData("L3 (Left Back)", getDirectionString(motor3Left));
        telemetry.addLine();

        telemetry.addLine("--- RIGHT SIDE DIRECTIONS ---");
        telemetry.addData("R1 (Right Front)", getDirectionString(motor1Right));
        telemetry.addData("R2 (Right Middle)", getDirectionString(motor2Right));
        telemetry.addData("R3 (Right Back)", getDirectionString(motor3Right));
        telemetry.addLine();

        telemetry.addLine("--- LEFT SIDE POWERS ---");
        telemetry.addData("L1", String.format(Locale.US, "%.2f", motor1Left.getPower()));
        telemetry.addData("L2", String.format(Locale.US, "%.2f", motor2Left.getPower()));
        telemetry.addData("L3", String.format(Locale.US, "%.2f", motor3Left.getPower()));
        telemetry.addLine();

        telemetry.addLine("--- RIGHT SIDE POWERS ---");
        telemetry.addData("R1", String.format(Locale.US, "%.2f", motor1Right.getPower()));
        telemetry.addData("R2", String.format(Locale.US, "%.2f", motor2Right.getPower()));
        telemetry.addData("R3", String.format(Locale.US, "%.2f", motor3Right.getPower()));
        telemetry.addLine();

        telemetry.addLine("GOAL: Find combination where:");
        telemetry.addLine("  A: Robot drives FORWARD");
        telemetry.addLine("  B: Robot turns RIGHT");
        telemetry.addLine("  X: Robot turns LEFT");
        telemetry.addLine();
        telemetry.addLine("Write down the combo number!");
    }
}