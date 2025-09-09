package org.firstinspires.ftc.teamcode.champion.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode demonstrates an automatic ball intake and launch system for an FTC robot.
 * The sequence runs automatically without gamepad input:
 * 1. Intake motor sucks in the ball
 * 2. Wait 3000ms (3 seconds)
 * 3. Launch motors launch the ball
 *
 * Hardware Configuration:
 * - Motor 1: Intake motor (name: "intakeMotor")
 * - Motor 2: Launch motor 1 (name: "launchMotor1")
 * - Motor 3: Launch motor 2 (name: "launchMotor2")
 */
@Autonomous(name="launchTest1")
public class intakeLaunchTest extends LinearOpMode {

    // Declare motor objects
    private DcMotor intakeMotor = null;
    private DcMotor launchMotor1 = null;
    private DcMotor launchMotor2 = null;

    // Timer for tracking elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Motor power constants - adjust these values based on your robot's needs
    private static final double INTAKE_POWER = 0.8;  // Power for intake motor (0.0 to 1.0)
    private static final double LAUNCH_POWER = 0.7;  // Power for launch motors (0.0 to 1.0)

    // Timing constants
    private static final double INTAKE_TIME_MS = 2000;  // Time to run intake motor (milliseconds)
    private static final double WAIT_TIME_MS = 3000;    // Wait time between intake and launch (milliseconds)
    private static final double LAUNCH_TIME_MS = 5000;  // Time to run launch motors (milliseconds)

    @Override
    public void runOpMode() {

        // Initialize hardware - map motor names to hardware configuration
        // IMPORTANT: These names must match your robot configuration file
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor1 = hardwareMap.get(DcMotor.class, "launchMotor1");
        launchMotor2 = hardwareMap.get(DcMotor.class, "launchMotor2");

        // Set motor directions
        // Adjust these based on how your motors are mounted
        // Use DcMotor.Direction.REVERSE if motor spins opposite to desired direction
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor1.setDirection(DcMotor.Direction.FORWARD);
        launchMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes to RUN_WITHOUT_ENCODER for direct power control
        // Change to RUN_USING_ENCODER if you have encoders and want velocity control
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to brake when power is zero (prevents coasting)
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            intakeMotor.setPower(INTAKE_POWER);

            // Run intake for specified duration
            // Adjust INTAKE_TIME_MS if ball needs more/less time to be secured
            sleep((long)INTAKE_TIME_MS);

            // Stop intake motor after ball is secured inside robot
            intakeMotor.setPower(0);

            telemetry.addData("Status", "Intake complete");
            telemetry.update();

            // ==============================================================
            // STEP 2: WAIT - Hold ball for 3000ms (3 seconds)
            // ==============================================================
            telemetry.addData("Status", "Step 2: Waiting 3 seconds before launch");
            telemetry.addData("Wait Time", "3.0 seconds");
            telemetry.update();

            // Wait for exactly 3000ms as specified
            sleep((long)WAIT_TIME_MS);

            // ==============================================================
            // STEP 3: LAUNCH - Launch the ball using motors 2 and 3
            // ==============================================================
            telemetry.addData("Status", "Step 3: Launching ball");
            telemetry.addData("Launch Power", "%.2f", LAUNCH_POWER);
            telemetry.addData("Duration", "%.1f seconds", LAUNCH_TIME_MS / 1000.0);
            telemetry.update();

            // Start both launch motors simultaneously at full power
            launchMotor1.setPower(LAUNCH_POWER);
            launchMotor2.setPower(LAUNCH_POWER);

            // Run launch motors for specified duration
            // Adjust LAUNCH_TIME_MS based on your launch mechanism requirements
            sleep((long)LAUNCH_TIME_MS);

            // Stop launch motors after ball is launched
            launchMotor1.setPower(0);
            launchMotor2.setPower(0);

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
        intakeMotor.setPower(0);
        launchMotor1.setPower(0);
        launchMotor2.setPower(0);

        telemetry.addData("Status", "All motors stopped - OpMode complete");
        telemetry.update();
    }
}