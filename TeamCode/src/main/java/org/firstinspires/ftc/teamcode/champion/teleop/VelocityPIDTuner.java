package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Velocity PID Tuner for Six Wheel Drive
 *
 * Tune the PIDF coefficients to get smooth, accurate velocity control.
 * Watch the graphs in FTC Dashboard to see how well the motors track the target.
 *
 * Controls:
 *   A = Run at 25% max velocity
 *   B = Run at 50% max velocity
 *   Y = Run at 75% max velocity
 *   X = Run at 100% max velocity
 *   Right Trigger = Variable speed (0-100%)
 *   Left Bumper = Reverse direction
 *   Right Bumper = Stop and apply new PID values
 *   DPad Up/Down = Fine tune target velocity
 *
 * TUNING PROCESS:
 * 1. Start with F term only (P=0, I=0, D=0)
 *    - F ≈ 32767 / MAX_TICKS_PER_SEC (typically 8-15)
 *    - Adjust until motor roughly reaches target
 * 2. Add P term (start at 10-20)
 *    - Increase until responsive but not oscillating
 * 3. Add D term if oscillating (start at 0.1-1.0)
 *    - Helps dampen overshoot
 * 4. Add I term only if steady-state error (start at 0.01)
 *    - Usually not needed
 */
@Config
@TeleOp(name = "Velocity PID Tuner", group = "Tuning")
public class VelocityPIDTuner extends LinearOpMode {

    // ========== MOTOR SPECS (goBILDA 312 RPM) ==========
    public static double TICKS_PER_REV = 751.8;
    public static double MAX_RPM = 312.0;
    public static double WHEEL_DIAMETER_MM = 68.5;  // Your wheel size

    // Calculated values
    public static double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;  // ~3909 ticks/sec
    public static double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;      // ~2.697 inches
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI; // ~8.47 inches
    public static double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;  // ~88.7 ticks/inch

    // ========== PID COEFFICIENTS (tune these in Dashboard) ==========
    public static double VELOCITY_P = 15.0;   // Proportional
    public static double VELOCITY_I = 0.0;    // Integral (usually 0)
    public static double VELOCITY_D = 0.5;    // Derivative
    public static double VELOCITY_F = 12.0;   // Feedforward (start here: 32767/MAX_TICKS_PER_SEC ≈ 8.4)

    // ========== TEST SETTINGS ==========
    public static double TARGET_VELOCITY_PERCENT = 50.0;  // % of max velocity
    public static boolean TEST_FORWARD = true;

    // Hardware
    private DcMotorEx lf, rf, lb, rb;

    // State
    private double targetVelocity = 0;
    private boolean isRunning = false;
    private boolean reverseDirection = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Set directions
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply initial PID
        applyPIDToAllMotors();

        telemetry.addLine("=== VELOCITY PID TUNER ===");
        telemetry.addLine();
        telemetry.addData("Max Velocity", "%.0f ticks/sec", MAX_TICKS_PER_SEC);
        telemetry.addData("Wheel Diameter", "%.2f in (%.1f mm)", WHEEL_DIAMETER_INCHES, WHEEL_DIAMETER_MM);
        telemetry.addLine();
        telemetry.addLine("A=25% | B=50% | Y=75% | X=100%");
        telemetry.addLine("RT=Variable | LB=Reverse | RB=Stop+Apply PID");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === BUTTON CONTROLS ===

            // Preset speeds
            if (gamepad1.a) {
                targetVelocity = MAX_TICKS_PER_SEC * 0.25;
                isRunning = true;
            } else if (gamepad1.b) {
                targetVelocity = MAX_TICKS_PER_SEC * 0.50;
                isRunning = true;
            } else if (gamepad1.y) {
                targetVelocity = MAX_TICKS_PER_SEC * 0.75;
                isRunning = true;
            } else if (gamepad1.x) {
                targetVelocity = MAX_TICKS_PER_SEC * 1.0;
                isRunning = true;
            }

            // Variable speed with right trigger
            if (gamepad1.right_trigger > 0.1) {
                targetVelocity = MAX_TICKS_PER_SEC * gamepad1.right_trigger;
                isRunning = true;
            }

            // Dashboard variable speed
            if (gamepad1.dpad_up) {
                TARGET_VELOCITY_PERCENT = Math.min(100, TARGET_VELOCITY_PERCENT + 1);
            } else if (gamepad1.dpad_down) {
                TARGET_VELOCITY_PERCENT = Math.max(0, TARGET_VELOCITY_PERCENT - 1);
            }

            // Left bumper = reverse
            if (gamepad1.left_bumper) {
                reverseDirection = !reverseDirection;
                sleep(200);  // Debounce
            }

            // Right bumper = stop and apply new PID
            if (gamepad1.right_bumper) {
                isRunning = false;
                targetVelocity = 0;
                stopAllMotors();
                applyPIDToAllMotors();
                sleep(200);  // Debounce
            }

            // Left trigger = stop
            if (gamepad1.left_trigger > 0.5) {
                isRunning = false;
                targetVelocity = 0;
            }

            // === APPLY VELOCITY ===
            double commandVelocity = isRunning ? targetVelocity : 0;
            if (reverseDirection) {
                commandVelocity = -commandVelocity;
            }

            // Set velocity to all motors
            lf.setVelocity(commandVelocity);
            lb.setVelocity(commandVelocity);
            rf.setVelocity(commandVelocity);
            rb.setVelocity(commandVelocity);

            // === READ ACTUAL VELOCITIES ===
            double lfVel = lf.getVelocity();
            double lbVel = lb.getVelocity();
            double rfVel = rf.getVelocity();
            double rbVel = rb.getVelocity();
            double avgVel = (lfVel + lbVel + rfVel + rbVel) / 4.0;

            // Calculate error
            double error = commandVelocity - avgVel;
            double errorPercent = (commandVelocity != 0) ? (error / commandVelocity * 100) : 0;

            // Convert to real-world units
            double targetInchesPerSec = (commandVelocity / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
            double actualInchesPerSec = (avgVel / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

            // === TELEMETRY (for Dashboard graphs) ===
            telemetry.addLine("=== TARGET vs ACTUAL ===");
            telemetry.addData("Target Velocity", "%.0f ticks/s", commandVelocity);
            telemetry.addData("Actual Velocity", "%.0f ticks/s", avgVel);
            telemetry.addData("Error", "%.0f ticks/s (%.1f%%)", error, errorPercent);
            telemetry.addLine();

            telemetry.addLine("=== REAL WORLD SPEED ===");
            telemetry.addData("Target Speed", "%.1f in/s", targetInchesPerSec);
            telemetry.addData("Actual Speed", "%.1f in/s", actualInchesPerSec);
            telemetry.addLine();

            telemetry.addLine("=== INDIVIDUAL MOTORS ===");
            telemetry.addData("LF Velocity", "%.0f", lfVel);
            telemetry.addData("LB Velocity", "%.0f", lbVel);
            telemetry.addData("RF Velocity", "%.0f", rfVel);
            telemetry.addData("RB Velocity", "%.0f", rbVel);
            telemetry.addLine();

            telemetry.addLine("=== PID COEFFICIENTS ===");
            telemetry.addData("P", "%.2f", VELOCITY_P);
            telemetry.addData("I", "%.4f", VELOCITY_I);
            telemetry.addData("D", "%.2f", VELOCITY_D);
            telemetry.addData("F", "%.2f", VELOCITY_F);
            telemetry.addLine();

            telemetry.addLine("=== STATUS ===");
            telemetry.addData("Running", isRunning);
            telemetry.addData("Direction", reverseDirection ? "REVERSE" : "FORWARD");
            telemetry.addData("Dashboard Target %", "%.0f%%", TARGET_VELOCITY_PERCENT);

            telemetry.update();
        }

        stopAllMotors();
    }

    private void applyPIDToAllMotors() {
        lf.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        lb.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        rf.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        rb.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
    }

    private void setAllMotorModes(DcMotor.RunMode mode) {
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    private void stopAllMotors() {
        lf.setVelocity(0);
        lb.setVelocity(0);
        rf.setVelocity(0);
        rb.setVelocity(0);
    }
}