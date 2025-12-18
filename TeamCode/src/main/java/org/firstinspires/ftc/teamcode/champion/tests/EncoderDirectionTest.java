package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.AutoTankDrive;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.PinpointLocalizer;

/**
 * Encoder Direction Diagnostic Test
 *
 * This OpMode helps diagnose encoder direction issues.
 *
 * Instructions:
 * 1. Run this OpMode
 * 2. Press A to drive forward at low power
 * 3. Check telemetry - ALL values should be POSITIVE when moving forward
 * 4. If any are negative, you need to fix encoder directions
 *
 * What to look for:
 * - Drive Motor Encoders: Should increase when driving forward
 * - Pinpoint X Encoder: Should change based on forward movement
 * - Pinpoint Y Encoder: Should increase significantly when driving forward
 *
 * Common Issues:
 * - If drive motors show negative: Motor directions may be wrong
 * - If Pinpoint encoders negative: Check Pinpoint encoder directions in config
 */
@TeleOp(name = "Encoder Direction Test", group = "Diagnostic")
public class EncoderDirectionTest extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private PinpointLocalizer pinpointLocalizer;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        // Set directions as in your AutoTankDrive
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set to run without encoders for this test
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Pinpoint
        pinpointLocalizer = new PinpointLocalizer(
                hardwareMap,
                AutoTankDrive.PARAMS.odoInPerTick,
                AutoTankDrive.PARAMS.pinpointYOffset,
                AutoTankDrive.PARAMS.pinpointXOffset,
                new com.acmerobotics.roadrunner.Pose2d(0, 0, 0)
        );

        telemetry.addLine("=== ENCODER DIRECTION TEST ===");
        telemetry.addLine();
        telemetry.addLine("Press A to drive FORWARD at low power");
        telemetry.addLine("ALL encoder values should be POSITIVE");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // Reset encoders at start
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpointLocalizer.driver.resetPosAndIMU();

        while (opModeIsActive()) {
            // Control with gamepad
            if (gamepad1.a) {
                // Drive forward at 30% power
                leftFront.setPower(0.3);
                leftBack.setPower(0.3);
                rightFront.setPower(0.3);
                rightBack.setPower(0.3);
            } else {
                // Stop
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }

            // Update Pinpoint
            pinpointLocalizer.update();

            // Get encoder positions
            int lfPos = leftFront.getCurrentPosition();
            int lbPos = leftBack.getCurrentPosition();
            int rfPos = rightFront.getCurrentPosition();
            int rbPos = rightBack.getCurrentPosition();

            // Get Pinpoint encoder values
            int pinpointX = pinpointLocalizer.driver.getEncoderX();
            int pinpointY = pinpointLocalizer.driver.getEncoderY();

            // Display results
            telemetry.addLine("=== DRIVE MOTOR ENCODERS ===");
            telemetry.addData("Left Front", "%d %s", lfPos, getStatus(lfPos));
            telemetry.addData("Left Back", "%d %s", lbPos, getStatus(lbPos));
            telemetry.addData("Right Front", "%d %s", rfPos, getStatus(rfPos));
            telemetry.addData("Right Back", "%d %s", rbPos, getStatus(rbPos));
            telemetry.addLine();

            telemetry.addLine("=== PINPOINT ODOMETRY ===");
            telemetry.addData("X Encoder", "%d %s", pinpointX, getStatus(pinpointX));
            telemetry.addData("Y Encoder", "%d %s", pinpointY, getStatus(pinpointY));
            telemetry.addLine();

            telemetry.addLine("=== INSTRUCTIONS ===");
            telemetry.addLine("Hold A to drive forward");
            telemetry.addLine("ALL values should show (OK) when moving forward");
            telemetry.addLine();

            // Overall status
            boolean allOk = lfPos >= 0 && lbPos >= 0 && rfPos >= 0 && rbPos >= 0 && pinpointY >= 0;

            if (gamepad1.a) {
                if (allOk) {
                    telemetry.addLine("✓ ALL ENCODERS OK - Good to proceed!");
                } else {
                    telemetry.addLine("✗ ENCODER ISSUES DETECTED");
                    telemetry.addLine();
                    if (lfPos < 0 || lbPos < 0 || rfPos < 0 || rbPos < 0) {
                        telemetry.addLine("→ Drive motor encoders negative!");
                        telemetry.addLine("  Check motor directions in code");
                    }
                    if (pinpointY < 0) {
                        telemetry.addLine("→ Pinpoint Y encoder negative!");
                        telemetry.addLine("  Check encoder directions in Pinpoint config");
                    }
                }
            }

            telemetry.update();
        }
    }

    private String getStatus(int value) {
        if (value > 10) {
            return "(OK)";
        } else if (value < -10) {
            return "(WRONG - NEGATIVE!)";
        } else {
            return "(waiting...)";
        }
    }
}