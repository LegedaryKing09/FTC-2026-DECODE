package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Motor Test", group = "Test")
public class DualMotorTest extends LinearOpMode {

    // Configure your motor name here
    public static String MOTOR1_NAME = "motor1";
    public static String MOTOR2_NAME = "motor2";

    // Set to true to run at full power automatically on start
    public static boolean AUTO_RUN_FULL_POWER = false;

    // Power level (set to 1.0 for full power)
    public static double TARGET_POWER = 1.0;

    private DcMotorEx motor1, motor2;
    private ElapsedTime runtime;
    private ElapsedTime testTimer;

    // Encoder position tracking for velocity calculation without MotorEx
    private int lastEncoderPosition1 = 0;
    private int lastEncoderPosition2 = 0;
    private long lastTimeMillis1 = 0;
    private long lastTimeMillis2 = 0;

    private boolean isPressingA = false;
    private boolean isPressingB = false;
    private boolean isPressingX = false;
    private boolean motorRunning = false;

    // Statistics
    private double maxVelocity = 0;
    private double avgVelocity = 0;
    private int sampleCount = 0;
    private double velocitySum = 0;

    // RPM conversion constants for FTC motors (ticks per second to RPM)
    private static final double TICKS_PER_REVOLUTION = 28.0; // Motor encoder specification
    private static final double SECONDS_PER_MINUTE = 60.0;

    private double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REVOLUTION) * SECONDS_PER_MINUTE;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return ticksToRPM(ticksPerSecond);
    }

    private double calculateVelocityRPM(DcMotor motor, int lastPosition, long lastTimeMillis, int motorIndex) {
        long currentTimeMillis = System.currentTimeMillis();
        int currentPosition = motor.getCurrentPosition();

        long timeDeltaMillis = currentTimeMillis - lastTimeMillis;
        if (timeDeltaMillis <= 0) return 0.0;

        int positionDelta = currentPosition - lastPosition;
        double ticksPerSecond = (positionDelta * 1000.0) / timeDeltaMillis; // Fixed calculation

        // Update tracking variables
        if (motorIndex == 1) {
            lastEncoderPosition1 = currentPosition;
            lastTimeMillis1 = currentTimeMillis;
        } else if (motorIndex == 2) {
            lastEncoderPosition2 = currentPosition;
            lastTimeMillis2 = currentTimeMillis;
        }

        return ticksPerSecondToRPM(ticksPerSecond);
    }

    private void initializeVelocityTracking() {
        lastEncoderPosition1 = motor1.getCurrentPosition();
        lastEncoderPosition2 = motor2.getCurrentPosition();
        lastTimeMillis1 = System.currentTimeMillis();
        lastTimeMillis2 = System.currentTimeMillis();
    }
    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotorEx.class, MOTOR1_NAME);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR2_NAME);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);


        runtime = new ElapsedTime();
        testTimer = new ElapsedTime();

        // Initialize velocity tracking
        initializeVelocityTracking();

        waitForStart();
        runtime.reset();

        // Auto-run if enabled
        if (AUTO_RUN_FULL_POWER && motor1 != null) {
            startMotor();
        }

        while (opModeIsActive()) {

            // A Button - Start motor at full power
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                startMotor();
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // B Button - Stop motor normally
            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                stopMotor();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            // X Button - Emergency stop
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                emergencyStop();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            // Update telemetry
            updateTelemetry();
        }

        // Clean shutdown
        if (motor1 != null) {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    private void startMotor() {
        if (motor1 != null) {
            motor1.setPower(TARGET_POWER);
            motor2.setPower(TARGET_POWER);
            motorRunning = true;
            testTimer.reset();
            maxVelocity = 0;
            avgVelocity = 0;
            sampleCount = 0;
            velocitySum = 0;


        }
    }

    private void stopMotor() {
        if (motor1 != null) {
            motor1.setPower(0);
            motor2.setPower(0);
            motorRunning = false;


        }
    }

    private void emergencyStop() {
        if (motor1 != null) {
            motor1.setPower(0);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setPower(0);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorRunning = false;


        }
    }

    private void updateTelemetry() {
        if (motor1 == null || motor2 == null) {
            telemetry.addData("ERROR", "Motors not initialized");
            telemetry.update();
            return;
        }

        // Get current motor data for motor1
        double currentPower1 = motor1.getPower();
        double vel1 = motor1.getVelocity();
        double velocity1 = ticksPerSecondToRPM(vel1);

        // Get current motor data for motor2
        double currentPower2 = motor2.getPower();
        double vel2 = motor2.getVelocity();
        double velocity2 = ticksPerSecondToRPM(vel2);

        // Update statistics (using motor1 as primary)
        if (motorRunning) {
            if (velocity1 > maxVelocity) {
                maxVelocity = velocity1;
            }
            velocitySum += velocity1;
            sampleCount++;
            avgVelocity = velocitySum / sampleCount;
        }

        telemetry.addData("Motor Status", motorRunning ? "RUNNING" : "STOPPED");

        // Power and control
        telemetry.addData("🎚️ Target Power", "%.3f (%.1f%%)", TARGET_POWER, TARGET_POWER * 100);
        telemetry.addData("⚡ Current Power1", "%.3f (%.1f%%)", currentPower1, currentPower1 * 100);
        telemetry.addData("⚡ Current Power2", "%.3f (%.1f%%)", currentPower2, currentPower2 * 100);

        // Performance metrics
        telemetry.addData("🔄 Velocity1", "%.2f RPM", velocity1);
        telemetry.addData("🔄 Velocity2", "%.2f RPM", velocity2);
        telemetry.addData("🔄 Vel1", "%.2f TPS", vel1);
        telemetry.addData("🔄 Vel2", "%.2f TPS", vel2);
        telemetry.addData("�📈 Max Velocity", "%.2f RPM", maxVelocity);
        telemetry.addData("📊 Avg Velocity", "%.2f RPM", avgVelocity);

        // Test duration
        if (motorRunning) {
            telemetry.addData("⏱️ Test Duration", "%.1f seconds", testTimer.seconds());
        }

        // Controls
        telemetry.addData("🟢 A Button", "Start Full Power");
        telemetry.addData("🔴 B Button", "Stop Motor");
        telemetry.addData("🛑 X Button", "Emergency Stop");

        // Warnings
        if (motorRunning && testTimer.seconds() > 30) {
            telemetry.addData("⚠️ WARNING", "Long test duration - monitor temperature!");
        }

        if (velocity1 > 5000) {
            telemetry.addData("⚠️ WARNING", "High RPM detected!");
        }

        telemetry.update();
    }
}