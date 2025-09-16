package org.firstinspires.ftc.teamcode.champion.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode demonstrates an automatic ball intake and launch system for an FTC robot.
 * The sequence runs automatically without gamepad input:
 * 1. Intake motor sucks in the ball
 * 2. Wait 1 second
 * 3. Shooter motor accelerates to optimal speed (2.5 seconds)
 * 4. Transfer and intake motors start while shooter maintains speed
 * 5. After 5 seconds, all motors stop
 * <p>
 * Hardware Configuration:
 * - Motor 1: Intake motor (name: "intake")
 * - Motor 2: Transfer motor(name: "transfer")
 * - Motor 3: Shooting motor (name: "shooter")
 */
@Autonomous(name="launchTest1")
public class intakeLaunchTest extends LinearOpMode {

    // Declare motor objects
    private DcMotor intake = null;
    private DcMotor transfer = null;
    private DcMotor shooter = null;

    // Timer for tracking elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    // Motor power constants - adjust these values based on your robot's needs
    private static final double INTAKE_POWER = -0.8;  // Power for intake motor (0.0 to 1.0)
    private static final double LAUNCH_POWER = -0.8;  // Power for launch motors (0.0 to 1.0)

    // Timing constants
    private static final double INTAKE_TIME_MS = 2000;      // Time to run intake motor (milliseconds)
    private static final double WAIT_TIME_MS = 1000;         // Wait time after intake (changed to 1 second)
    private static final double SHOOTER_SPINUP_TIME_MS = 2500; // Time for shooter to reach optimal speed (2.5 seconds)
    private static final double FINAL_LAUNCH_TIME_MS = 5000;   // Time to run all motors together (5 seconds)

    @Override
    public void runOpMode() {

        // Initialize hardware - map motor names to hardware configuration
        // IMPORTANT: These names must match your robot configuration file
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Set motor directions
        // Adjust these based on how your motors are mounted
        // Use DcMotor.Direction.REVERSE if motor spins opposite to desired direction
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes to RUN_WITHOUT_ENCODER for direct power control
        // Change to RUN_USING_ENCODER if you have encoders and want velocity control
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to brake when power is zero (prevents coasting)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate robot is ready
        telemetry.addData("Status", "Initialized - Press START to begin sequence");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Check if OpMode is still active before starting sequence
        if (opModeIsActive()) {

            // ==============================================================
            // AUTOMATIC INTAKE AND LAUNCH SEQUENCE
            // ==============================================================

            // STEP 1: INTAKE - Suck in the ball
            // ---------------------------------
            telemetry.addData("Status", "Step 1: Running intake motor");
            telemetry.addData("Intake Power", "%.2f", INTAKE_POWER);
            telemetry.addData("Duration", "%.1f seconds", INTAKE_TIME_MS / 1000.0);
            telemetry.update();

            // Start intake motor to suck in the ball
            intake.setPower(INTAKE_POWER);

            // Run intake for specified duration (keeping original intake time)
            sleep((long)INTAKE_TIME_MS);

            // Stop intake motor after ball is secured inside robot
            intake.setPower(0);

            telemetry.addData("Status", "Intake complete");
            telemetry.update();

            // ==============================================================
            // STEP 2: WAIT - Hold ball for 1 second
            // ==============================================================
            telemetry.addData("Status", "Step 2: Waiting 1 second");
            telemetry.addData("Wait Time", "1.0 second");
            telemetry.update();

            // Wait for 1 second as specified
            sleep((long)WAIT_TIME_MS);

            // ==============================================================
            // STEP 3: SHOOTER SPIN-UP - Accelerate shooter to optimal speed
            // ==============================================================
            telemetry.addData("Status", "Step 3: Spinning up shooter motor");
            telemetry.addData("Shooter Power", "%.2f", LAUNCH_POWER);
            telemetry.addData("Spin-up Duration", "2.5 seconds");
            telemetry.update();

            // Start shooter motor to reach optimal speed
            shooter.setPower(LAUNCH_POWER);

            // Allow shooter to accelerate for 2.5 seconds
            sleep((long)SHOOTER_SPINUP_TIME_MS);

            // ==============================================================
            // STEP 4: LAUNCH - Start transfer and intake while maintaining shooter
            // ==============================================================
            telemetry.addData("Status", "Step 4: Launching ball");
            telemetry.addData("Transfer Power", "1.0");
            telemetry.addData("Intake Power", "1.0");
            telemetry.addData("Shooter Power", "%.2f (maintained)", LAUNCH_POWER);
            telemetry.addData("Duration", "5.0 seconds");
            telemetry.update();

            // Start transfer and intake motors at full power (1.0)
            // Shooter continues at same speed (LAUNCH_POWER)
            transfer.setPower(-1.0);
            intake.setPower(-1.0);
            // Note: Shooter is already running at LAUNCH_POWER, no need to set again

            // Run all motors together for 5 seconds
            sleep((long)FINAL_LAUNCH_TIME_MS);

            // ==============================================================
            // STEP 5: STOP - Stop all motors
            // ==============================================================

            // Stop all motors after launch sequence
            transfer.setPower(0);
            shooter.setPower(0);
            intake.setPower(0);

            // ==============================================================
            // SEQUENCE COMPLETE
            // ==============================================================
            telemetry.addData("Status", "Sequence Complete!");
            telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
            telemetry.addData("Result", "Ball has been launched successfully");
            telemetry.update();

            // Keep telemetry displayed for 3 seconds before ending
            sleep(3000);
        }

        // Ensure all motors are stopped before OpMode ends
        stopAllMotors();
    }

    /**
     * Emergency stop method - stops all motors immediately
     * This is called at the end of the sequence or if OpMode is stopped early
     */
    private void stopAllMotors() {
        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);

        telemetry.addData("Status", "All motors stopped - OpMode complete");
        telemetry.update();
    }
}