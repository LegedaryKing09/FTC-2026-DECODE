package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@Config
@TeleOp(name = "Eight Wheel Sensitivity Test")
public class EightWheelDriveTest extends LinearOpMode {

    // Motor configuration names
    public static String L1_NAME = "l1";
    public static String L2_NAME = "l2";
    public static String L3_NAME = "l3";
    public static String R1_NAME = "r1";
    public static String R2_NAME = "r2";
    public static String R3_NAME = "r3";


    // Speed multipliers
    public static double FAST_SPEED_MULTIPLIER = 1;
    public static double FAST_TURN_MULTIPLIER = 1;
    public static double SLOW_SPEED_MULTIPLIER = 0.4;
    public static double SLOW_TURN_MULTIPLIER = 0.3;

    // Motor references
    private DcMotor motor1Left, motor2Left, motor3Left;
    private DcMotor motor1Right, motor2Right, motor3Right;
    boolean isPressingA = false;
    private boolean isFastSpeedMode = false;

    @Override
    public void runOpMode() {
        // Initialize motors
        motor1Left = hardwareMap.get(DcMotor.class, L1_NAME);
        motor2Left = hardwareMap.get(DcMotor.class, L2_NAME);
        motor3Left = hardwareMap.get(DcMotor.class, L3_NAME);
        motor1Right = hardwareMap.get(DcMotor.class, R1_NAME);
        motor2Right = hardwareMap.get(DcMotor.class, R2_NAME);
        motor3Right = hardwareMap.get(DcMotor.class, R3_NAME);

        // Set motor directions - UPDATE THESE BASED ON YOUR TEST RESULTS
        // EXAMPLE - Replace with your tested combination:
        motor1Left.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2Left.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3Left.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1Right.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2Right.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3Right.setDirection(DcMotorSimple.Direction.REVERSE);

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

        telemetry.addLine("Eight Wheel Sensitivity Test Ready");
        telemetry.addLine("Left stick: Drive");
        telemetry.addLine("Right stick: Turn");
        telemetry.addLine("Back button: Toggle speed mode");
        telemetry.addLine();
        telemetry.addLine("Sensitivity curves make low-speed");
        telemetry.addLine("control smoother and more precise!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get raw joystick inputs
            double rawDrive = -gamepad1.left_stick_y;
            double rawTurn = -gamepad1.right_stick_x;

            // Apply speed multipliers based on mode
            double drive, turn;
            if (isFastSpeedMode) {
                drive = rawDrive * FAST_SPEED_MULTIPLIER;
                turn = rawTurn * FAST_TURN_MULTIPLIER;
            } else {
                drive = rawDrive * SLOW_SPEED_MULTIPLIER;
                turn = rawTurn * SLOW_TURN_MULTIPLIER;
            }

            // Calculate left and right power (arcade drive)
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Normalize powers if any exceed 1.0
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            // Set motor powers
            motor1Left.setPower(leftPower);
            motor2Left.setPower(leftPower);
            motor3Left.setPower(leftPower);
            motor1Right.setPower(rightPower);
            motor2Right.setPower(rightPower);
            motor3Right.setPower(rightPower);

            // Handle speed mode toggle
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                isFastSpeedMode = !isFastSpeedMode;
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // Display telemetry
            displayTelemetry(rawDrive, rawTurn, leftPower, rightPower);
            telemetry.update();
        }
    }

    /**
     * Apply deadzone to joystick input - values below threshold become 0
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        // Re-scale the value to maintain smooth transition
        // When value = deadzone, output = 0
        // When value = 1.0, output = 1.0
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double scaled = (magnitude - deadzone) / (1.0 - deadzone);
        return sign * scaled;
    }

    /**
     * Apply sensitivity curve to input
     * Exponent > 1.0 makes the joystick less sensitive at low values
     * This gives better fine control at low speeds
     */
    private double applySensitivityCurve(double value, double exponent) {
        // Preserve sign, apply curve to magnitude
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double curved = Math.pow(magnitude, exponent);
        return sign * curved;
    }

    private void displayTelemetry(double rawDrive, double rawTurn,
                                  double leftPower, double rightPower) {
        telemetry.addLine("=== EIGHT WHEEL SENSITIVITY TEST ===");
        telemetry.addLine();

        // Speed mode
        telemetry.addData("Speed Mode", isFastSpeedMode ? "FAST" : "SLOW");
        telemetry.addLine();

        // Sensitivity settings
        telemetry.addLine("--- SENSITIVITY SETTINGS ---");
        telemetry.addLine();

        // Raw vs processed inputs
        telemetry.addLine("--- JOYSTICK PROCESSING ---");
        telemetry.addData("Raw Drive", String.format(Locale.US, "%.2f", rawDrive));
        telemetry.addData("Raw Turn", String.format(Locale.US, "%.2f", rawTurn));
        telemetry.addLine();

        // Final motor commands
        telemetry.addLine("--- MOTOR POWERS ---");
        telemetry.addData("Left Target", String.format(Locale.US, "%.2f", leftPower));
        telemetry.addData("Right Target", String.format(Locale.US, "%.2f", rightPower));
        telemetry.addLine();
        telemetry.addData("L1", String.format(Locale.US, "%.2f", motor1Left.getPower()));
        telemetry.addData("L2", String.format(Locale.US, "%.2f", motor2Left.getPower()));
        telemetry.addData("L3", String.format(Locale.US, "%.2f", motor3Left.getPower()));
        telemetry.addData("R1", String.format(Locale.US, "%.2f", motor1Right.getPower()));
        telemetry.addData("R2", String.format(Locale.US, "%.2f", motor2Right.getPower()));
        telemetry.addData("R3", String.format(Locale.US, "%.2f", motor3Right.getPower()));
        telemetry.addLine();
        telemetry.addLine("TIP: Adjust curves in FTC Dashboard");
        telemetry.addLine("for smoother low-speed control!");
    }
}