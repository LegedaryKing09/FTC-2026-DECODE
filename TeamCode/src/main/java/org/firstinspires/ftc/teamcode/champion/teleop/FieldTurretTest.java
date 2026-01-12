package org.firstinspires.ftc.teamcode.champion.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretFieldController;

/**
 * Field-Centric Turret Test with PID Tuning
 * Drive the robot while the turret automatically maintains aim at a fixed field direction.
 * The turret angle is set to 0° ONCE at startup and remains absolute throughout.
 * SETUP:
 * 1. Position robot at starting location
 * 2. Point turret at the target (this will be 0° turret angle)
 * 3. Press Start
 * 4. Set TARGET_FIELD_ANGLE = 0° (since turret starts pointing at target)
 * 5. Drive around - turret should stay aimed at target!
 * ALTERNATIVE SETUP:
 * 1. Position robot, turret centered on robot
 * 2. Measure angle from robot forward to target (e.g., -30° if target is right)
 * 3. Set TARGET_FIELD_ANGLE = -30°
 * Controls:
 *   Left Stick = Drive forward/back
 *   Right Stick = Turn robot
 *   A = Enable field-centric mode
 *   B = Disable (manual turret mode)
 *   X = Manual turret left (CCW)
 *   Y = Manual turret right (CW)
 *   DPad Up/Down = Adjust field target ±10°
 *   DPad Left/Right = Adjust field target ±1°
 *   Left Bumper = Reset IMU only (NOT turret!)
 *   Right Bumper = Toggle drive speed
 * NOTE: There is NO way to reset turret angle during operation!
 * This is intentional - turret angle must stay absolute.
 */
@Config
@TeleOp(name = "Field Centric Turret Test", group = "Test")
public class FieldTurretTest extends LinearOpMode {

    public static double TARGET_FIELD_ANGLE = 0.0;
    public static double MANUAL_TURRET_POWER = 0.4;

    private SixWheelDriveController drive;
    private TurretController turret;
    private TurretFieldController fieldController;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize controllers
        drive = new SixWheelDriveController(this);
        turret = new TurretController(this);
        fieldController = new TurretFieldController(turret);
        waitForStart();

        // === INITIALIZE ONCE AT START ===
        drive.resetOdometry();      // IMU = 0°
        turret.initialize();        // Turret = 0° (ONLY TIME THIS IS CALLED!)

        fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
        fieldController.enable();

        boolean wasPressingDPadUp = false;
        boolean wasPressingDPadDown = false;
        boolean wasPressingDPadLeft = false;
        boolean wasPressingDPadRight = false;
        boolean wasPressingLB = false;
        boolean wasPressingRB = false;

        while (opModeIsActive()) {
            // === UPDATE SENSORS (every loop!) ===
            drive.updateOdometry();
            turret.update();  // Track angle wraparound

            double robotHeadingDeg = drive.getHeadingDegrees();

            // === DRIVETRAIN ===
            double driveInput = -gamepad1.left_stick_y;
            double turnInput = gamepad1.right_stick_x;

            double speedMult = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_SPEED_MULTIPLIER :
                    SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
            double turnMult = drive.isFastSpeedMode() ?
                    SixWheelDriveController.FAST_TURN_MULTIPLIER :
                    SixWheelDriveController.SLOW_TURN_MULTIPLIER;

            drive.arcadeDrive(driveInput * speedMult, turnInput * turnMult);

            // === MODE CONTROL ===
            if (gamepad1.a) {
                fieldController.enable();
            }
            if (gamepad1.b) {
                fieldController.disable();
            }

            // === ADJUST TARGET ===
            if (gamepad1.dpad_up && !wasPressingDPadUp) {
                TARGET_FIELD_ANGLE += 10.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            wasPressingDPadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !wasPressingDPadDown) {
                TARGET_FIELD_ANGLE -= 10.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            wasPressingDPadDown = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !wasPressingDPadRight) {
                TARGET_FIELD_ANGLE += 1.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            wasPressingDPadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !wasPressingDPadLeft) {
                TARGET_FIELD_ANGLE -= 1.0;
                fieldController.setTargetFieldAngle(TARGET_FIELD_ANGLE);
            }
            wasPressingDPadLeft = gamepad1.dpad_left;

            // === RESET IMU ONLY (not turret!) ===
            if (gamepad1.left_bumper && !wasPressingLB) {
                drive.resetOdometry();
                fieldController.resetPID();  // Reset PID state, NOT turret angle
            }
            wasPressingLB = gamepad1.left_bumper;

            // === SPEED MODE ===
            if (gamepad1.right_bumper && !wasPressingRB) {
                drive.toggleSpeedMode();
            }
            wasPressingRB = gamepad1.right_bumper;

            // === TURRET CONTROL ===
            double turretPower = 0;
            if (fieldController.isEnabled()) {
                // Field-centric auto control with PID
                turretPower = fieldController.update(robotHeadingDeg);
            } else {
                // Manual control
                if (gamepad1.y) {
                    turretPower = MANUAL_TURRET_POWER;
                    turret.setPower(turretPower);
                } else if (gamepad1.x) {
                    turretPower = -MANUAL_TURRET_POWER;
                    turret.setPower(turretPower);
                } else {
                    turret.stop();
                }
            }

            // === TELEMETRY ===
            telemetry.addLine("=== FIELD CENTRIC TURRET ===");
            String mode = "MANUAL";
            if (fieldController.isEnabled()) {
                if (fieldController.isUnwrapping()) {
                    mode = "UNWRAPPING (WIRE SAFETY)";
                } else {
                    mode = "FIELD-CENTRIC (PID)";
                }
            }
            telemetry.addData("Mode", mode);
            telemetry.addData("Aligned", fieldController.isAligned() ? "✓ YES" : "NO");
            telemetry.addLine();

            // Field tracking (most important!)
            telemetry.addLine("=== FIELD TRACKING ===");
            telemetry.addData("Target Field", "%.1f°", TARGET_FIELD_ANGLE);
            telemetry.addData("Current Field", "%.1f°", fieldController.getCurrentFieldFacing());
            telemetry.addData("Field Error", "%.1f°", fieldController.getFieldError());
            telemetry.addLine();

            // The math
            telemetry.addLine("=== THE MATH ===");
            telemetry.addData("Robot Heading", "%.1f°", robotHeadingDeg);
            telemetry.addData("+ Turret Angle", "%.1f°", turret.getTurretAngle());
            telemetry.addData("= Field Facing", "%.1f°", fieldController.getCurrentFieldFacing());
            telemetry.addLine();

            // Turret state
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Turret Angle (absolute)", "%.1f°", turret.getTurretAngle());
            if (fieldController.isUnwrapping()) {
                telemetry.addData("Unwrap Target", "%.1f°", fieldController.getUnwrapTarget());
                telemetry.addData("Unwrap Progress", "%.1f°",
                    Math.abs(fieldController.getUnwrapTarget() - turret.getTurretAngle()));
            } else {
                telemetry.addData("Turret Target", "%.1f°", fieldController.getCalculatedTurretTarget());
            }
            telemetry.addData("Power", "%.3f", turretPower);
            telemetry.addLine();

            // PID tuning
            telemetry.addLine("=== FIELD PID ===");
            telemetry.addData("INVERT_OUTPUT", TurretFieldController.INVERT_OUTPUT);
            telemetry.addLine();

            // Wire safety status
            telemetry.addLine("=== WIRE SAFETY ===");
            telemetry.addData("Enabled", TurretFieldController.USE_WIRE_SAFETY);
            telemetry.addData("Threshold", "±%.1f°", TurretFieldController.WIRE_SAFETY_THRESHOLD);
            double safetyMargin = TurretFieldController.WIRE_SAFETY_THRESHOLD - Math.abs(turret.getTurretAngle());
            @SuppressLint("DefaultLocale") String safetyStatus = safetyMargin > 0 ? String.format("SAFE (%.1f° margin)", safetyMargin) : "⚠ UNSAFE";
            telemetry.addData("Status", safetyStatus);
            telemetry.addLine();

            // Angle tracking debug
            telemetry.addLine("=== ANGLE TRACKING ===");
            telemetry.addData("Servo Rotations", turret.getServoRotationCount());
            telemetry.addData("Raw Servo", "%.1f°", turret.getRawServoAngle());
            telemetry.addLine();

            // Robot
            telemetry.addData("Speed", drive.isFastSpeedMode() ? "FAST" : "SLOW");
            telemetry.addLine("A=Auto | B=Manual | LB=ResetIMU | RB=Speed");

            telemetry.update();
        }

        drive.stopDrive();
        turret.stop();
    }
}