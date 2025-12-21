package org.firstinspires.ftc.teamcode.champion.Auton.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;

/**
 * Manual Feedforward Tuner (kA)
 *
 * This OpMode runs the robot forward and backward repeatedly to tune kA (acceleration gain).
 * The goal is to make actual velocity match the target velocity profile.
 * Instructions:
 * 1. Set DISTANCE (how far to drive, e.g., 60 inches)
 * 2. Ensure you have clear space (DISTANCE × 2 + margin)
 * 3. Open Dashboard: http://192.168.43.1:8080/dash
 * 4. Press START
 * 5. Robot will drive forward/backward repeatedly
 * 6. Graph "vref" (target velocity) vs "v0" (actual velocity)
 * 7. Start with kA = 0.0000001, increase by 10× until it affects the plot
 * 8. Adjust kA until the two lines match closely
 *
 * Controls:
 * - Y/△ (Xbox/PS4): Pause and enter driver override to reposition robot
 * - B/○ (Xbox/PS4): Resume tuning
 */
@Config
@Autonomous(name = "Manual Feedforward Tuner (kA)", group = "Tuning")
public class kATest extends LinearOpMode {

    // Tunable parameters (appear on FTC Dashboard)
    public static double DISTANCE = 48; // Distance to travel in inches
    public static double MAX_VEL = 30; // Max velocity in inches/sec
    public static double MAX_ACCEL = 30; // Max acceleration in inches/sec²

    // You can adjust kA live on the dashboard
    public static double kA = 0.0;

    private boolean isPaused = false;
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        // Set directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure encoders for velocity feedback
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get voltage sensor
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Setup data logger
        DownsampledWriter writer = new DownsampledWriter("MANUAL_FF_TUNER", 50_000_000);

        telemetry.addLine("==================================");
        telemetry.addLine("FEEDFORWARD TUNER (kA)");
        telemetry.addLine("==================================");
        telemetry.addLine();
        telemetry.addData("Distance", "%.1f inches", DISTANCE);
        telemetry.addData("Space needed", "%.1f inches (forward + back)", DISTANCE * 2 + 24);
        telemetry.addLine();
        telemetry.addLine("Robot will drive forward & backward");
        telemetry.addLine("repeatedly until you stop it.");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Y/△: Pause & drive manually");
        telemetry.addLine("  B/○: Resume tuning");
        telemetry.addLine();
        telemetry.addLine("Open Dashboard to tune kA:");
        telemetry.addLine("http://192.168.43.1:8080/dash");
        telemetry.addLine();
        telemetry.addLine("Graph 'vref' vs 'v0' and adjust kA");
        telemetry.addLine("until the lines match.");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        int runNumber = 0;
        boolean goingForward = true;

        while (opModeIsActive()) {
            // Handle pause/resume controls
            if (gamepad1.y && !previousGamepad1.y) {
                isPaused = true;
                telemetry.addLine("⏸ PAUSED - Driver Override Active");
                telemetry.addLine("Use gamepad to reposition robot");
                telemetry.addLine("Press B/○ to resume tuning");
                telemetry.update();
            }
            if (gamepad1.b && !previousGamepad1.b) {
                isPaused = false;
            }
            previousGamepad1.copy(gamepad1);

            // Driver override mode
            if (isPaused) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                leftFront.setPower(drive + turn);
                leftBack.setPower(drive + turn);
                rightFront.setPower(drive - turn);
                rightBack.setPower(drive - turn);

                sleep(10);
                continue;
            }

            runNumber++;
            String direction = goingForward ? "FORWARD" : "BACKWARD";
            double targetDistance = goingForward ? DISTANCE : -DISTANCE;

            telemetry.clearAll();
            telemetry.addLine("==================================");
            telemetry.addLine("RUN #" + runNumber + " - " + direction);
            telemetry.addLine("==================================");
            telemetry.addData("kA (current)", "%.8f", kA);
            telemetry.addLine();
            telemetry.addLine("Press Y/△ to pause & reposition");
            telemetry.update();

            // Reset position tracking
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double startTime = System.nanoTime() / 1e9;
            double lastTime = startTime;

            // Generate trapezoidal motion profile
            TrapezoidProfile profile = new TrapezoidProfile(Math.abs(targetDistance), MAX_VEL, MAX_ACCEL);

            while (opModeIsActive() && !isPaused) {
                double currentTime = System.nanoTime() / 1e9;
                double elapsedTime = currentTime - startTime;
                double dt = currentTime - lastTime;

                if (dt < 0.001) {
                    continue; // Skip if too little time has passed
                }

                // Get profile state
                ProfileState state = profile.calculate(elapsedTime);

                // Check if profile is complete
                if (elapsedTime > profile.totalTime) {
                    break;
                }

                // Apply direction to profile
                double signedVelocity = goingForward ? state.velocity : -state.velocity;
                double signedAcceleration = goingForward ? state.acceleration : -state.acceleration;

                // Get current velocities (ticks per second)
                double lfVel = leftFront.getVelocity();
                double lbVel = leftBack.getVelocity();
                double rfVel = rightFront.getVelocity();
                double rbVel = rightBack.getVelocity();

                // Flip right side velocities (they're reversed)
                rfVel = -rfVel;
                rbVel = -rbVel;

                // Calculate average velocity in ticks/sec
                double avgVelTicks = (lfVel + lbVel + rfVel + rbVel) / 4.0;

                // Convert to inches/sec for display
                double actualVel = avgVelTicks * AutoTankDrive.PARAMS.inPerTick;

                // Calculate feedforward using current kA
                MotorFeedforward feedforward = new MotorFeedforward(
                        AutoTankDrive.PARAMS.kS,
                        AutoTankDrive.PARAMS.kV / AutoTankDrive.PARAMS.inPerTick,
                        kA / AutoTankDrive.PARAMS.inPerTick
                );

                double targetVelTicks = signedVelocity / AutoTankDrive.PARAMS.inPerTick;
                double targetAccelTicks = signedAcceleration / AutoTankDrive.PARAMS.inPerTick;

                // Calculate motor voltage
                double voltage = voltageSensor.getVoltage();
                double ffVoltage = feedforward.compute(targetVelTicks, targetAccelTicks);
                double power = ffVoltage / voltage;

                // Clamp power to safe range
                power = Math.max(-1.0, Math.min(1.0, power));

                // Apply power
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(power);

                // Log data with "vref" and "v0" names for dashboard graphing
                writer.write(new FeedforwardDataMessage(
                        elapsedTime,
                        state.position,
                        signedVelocity,      // vref (target velocity)
                        actualVel,           // v0 (actual velocity)
                        signedAcceleration,
                        power,
                        voltage,
                        kA
                ));

                // Update telemetry
                telemetry.clearAll();
                telemetry.addLine("RUN #" + runNumber + " - " + direction);
                telemetry.addLine();
                telemetry.addData("Progress", "%.1f%%", (elapsedTime / profile.totalTime) * 100);
                telemetry.addLine();
                telemetry.addData("kA", "%.8f", kA);
                telemetry.addLine();
                telemetry.addData("vref (Target Vel)", "%.2f in/s", signedVelocity);
                telemetry.addData("v0 (Actual Vel)", "%.2f in/s", actualVel);
                telemetry.addData("Velocity Error", "%.2f in/s", signedVelocity - actualVel);
                telemetry.addLine();
                telemetry.addData("Target Accel", "%.2f in/s²", signedAcceleration);
                telemetry.addLine();
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("Voltage", "%.2f V", voltage);
                telemetry.addLine();
                telemetry.addLine("Tuning Guide:");
                double velError = Math.abs(signedVelocity - actualVel);
                if (velError > 2.0) {
                    if (signedVelocity - actualVel > 0) {
                        telemetry.addLine("→ Actual lags: INCREASE kA");
                    } else {
                        telemetry.addLine("→ Actual overshoots: DECREASE kA");
                    }
                } else {
                    telemetry.addLine("✓ Velocities match well!");
                }
                telemetry.addLine();
                telemetry.addLine("Press Y/△ to pause");
                telemetry.update();

                lastTime = currentTime;

                sleep(10);
            }

            // Stop motors
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Switch direction for next run
            goingForward = !goingForward;

            // Short pause between runs
            sleep(500);
        }
    }

    // Trapezoidal motion profile generator
    static class TrapezoidProfile {
        double distance;
        double maxVel;
        double maxAccel;
        double accelTime;
        double cruiseTime;
        double decelTime;
        double totalTime;
        double accelDist;
        double cruiseDist;
        double decelDist;

        public TrapezoidProfile(double distance, double maxVel, double maxAccel) {
            this.distance = distance;
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;

            // Calculate profile timing
            accelTime = maxVel / maxAccel;
            accelDist = 0.5 * maxAccel * accelTime * accelTime;

            if (accelDist * 2 > distance) {
                // Triangular profile (no cruise phase)
                accelDist = distance / 2;
                accelTime = Math.sqrt(2 * accelDist / maxAccel);
                maxVel = maxAccel * accelTime;
                cruiseTime = 0;
                cruiseDist = 0;
            } else {
                // Trapezoidal profile
                cruiseDist = distance - 2 * accelDist;
                cruiseTime = cruiseDist / maxVel;
            }

            decelTime = accelTime;
            decelDist = accelDist;
            totalTime = accelTime + cruiseTime + decelTime;
        }

        public ProfileState calculate(double time) {
            ProfileState state = new ProfileState();

            if (time < accelTime) {
                // Acceleration phase
                state.position = 0.5 * maxAccel * time * time;
                state.velocity = maxAccel * time;
                state.acceleration = maxAccel;
            } else if (time < accelTime + cruiseTime) {
                // Cruise phase
                double cruiseElapsed = time - accelTime;
                state.position = accelDist + maxVel * cruiseElapsed;
                state.velocity = maxVel;
                state.acceleration = 0;
            } else if (time < totalTime) {
                // Deceleration phase
                double decelElapsed = time - accelTime - cruiseTime;
                state.position = accelDist + cruiseDist + maxVel * decelElapsed - 0.5 * maxAccel * decelElapsed * decelElapsed;
                state.velocity = maxVel - maxAccel * decelElapsed;
                state.acceleration = -maxAccel;
            } else {
                // Complete
                state.position = distance;
                state.velocity = 0;
                state.acceleration = 0;
            }

            return state;
        }
    }

    static class ProfileState {
        double position;
        double velocity;
        double acceleration;
    }

    // Data message for logging - using vref and v0 naming convention
    public static class FeedforwardDataMessage {
        public final double time;
        public final double targetPosition;
        public final double vref;  // Target velocity (for dashboard graphing)
        public final double v0;    // Actual velocity (for dashboard graphing)
        public final double targetAcceleration;
        public final double power;
        public final double voltage;
        public final double kA;

        public FeedforwardDataMessage(double time, double targetPosition,
                                      double vref, double v0,
                                      double targetAcceleration,
                                      double power, double voltage, double kA) {
            this.time = time;
            this.targetPosition = targetPosition;
            this.vref = vref;
            this.v0 = v0;
            this.targetAcceleration = targetAcceleration;
            this.power = power;
            this.voltage = voltage;
            this.kA = kA;
        }
    }
}