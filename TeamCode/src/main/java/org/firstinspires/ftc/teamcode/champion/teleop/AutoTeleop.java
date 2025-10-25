package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;

@Config
@TeleOp(name = "Enhanced TeleOp with Ramp", group = "Competition")
public class AutoTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;

    public static double SHOOTING_POWER = 0.55;
    public static double INTAKE_POWER = 0;
    public static double RAMP_INCREMENT = 2.5;  // Manual ramp control increment

    boolean isManualAligning = false;
    boolean last_dpadLeft = false;
    boolean last_dpadRight = false;
    boolean isUsingTelemetry = true;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingX = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingRightTrigger = false;
    boolean isPressingStart = false;
    boolean isPressingDpadDown = false;
    boolean isPressingDpadUp = false;
    boolean isPressingBack = false;
    boolean isPressingLeftStickButton = false;
    boolean isPressingRightStickButton = false;

    @Override
    public void runOpMode() {

        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);
        rampController = new RampController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LimelightAlignmentController tempLimelight = null;
        try {
            // FIX: Pass both opMode (this) and driveController
            tempLimelight = new LimelightAlignmentController(this, driveController);
            tempLimelight.setTargetTag(AutoShootController.APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }
        limelightController = tempLimelight;


        // Initialize enhanced auto shoot controller with ramp controller (7 parameters now)
        autoShootController = new AutoShootController(
                this,
                driveController,
                shooterController,
                intakeController,
                transferController,
                limelightController,
                rampController
        );

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

        // Initialize ramp to starting position
        rampController.setTo0Degrees();

        waitForStart();

        while (opModeIsActive()) {

            shooterController.updatePID();

            if (driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.FAST_TURN_MULTIPLIER;
            }
            if (!driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;
            }

            driveController.arcadeDrive(drive, turn);

            // A button: Set to FAST speed mode
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                driveController.setFastSpeed();
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // B button: Set to SLOW speed mode
            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                driveController.setSlowSpeed();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            // Y button: Set shooter to custom power (calculated distance RPM)
            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                shooterController.setShooterPower(SHOOTING_POWER);
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            // X button: Stop all systems
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                shooterController.shooterStop();
                intakeController.intakeStop();
                transferController.transferStop();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            // D-pad UP: Increase shooting power
            if (gamepad1.dpad_up && SHOOTING_POWER < 1 && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_POWER = SHOOTING_POWER + 0.01;
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            // D-pad DOWN: Decrease shooting power
            if (gamepad1.dpad_down && SHOOTING_POWER > 0 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_POWER = SHOOTING_POWER - 0.01;
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }

            // Right trigger: Transfer full
            if (gamepad1.right_trigger > 0.1 && !isPressingRightTrigger) {
                isPressingRightTrigger = true;
                transferController.transferFull();
            } else if (gamepad1.right_trigger < 0.1 && isPressingRightTrigger) {
                isPressingRightTrigger = false;
                transferController.transferFull();
            }

            // Right bumper: Intake full
            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;
                intakeController.intakeFull();
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
                intakeController.intakeStop();
            }

            // Left bumper: Intake eject
            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;
                intakeController.intakeEject();
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
                intakeController.intakeStop();
            }

            // Left stick button: Manual ramp retract (decrease angle)
            if (gamepad1.left_stick_button && !isPressingLeftStickButton) {
                isPressingLeftStickButton = true;
                rampController.decrementAngle(RAMP_INCREMENT);
            } else if (!gamepad1.left_stick_button && isPressingLeftStickButton) {
                isPressingLeftStickButton = false;
            }

            // Right stick button: Manual ramp extend (increase angle)
            if (gamepad1.right_stick_button && !isPressingRightStickButton) {
                isPressingRightStickButton = true;
                rampController.incrementAngle(RAMP_INCREMENT);
            } else if (!gamepad1.right_stick_button && isPressingRightStickButton) {
                isPressingRightStickButton = false;
            }

            // Back button: Toggle speed mode (kept for redundancy)
            if (gamepad1.back && !isPressingBack) {
                isPressingBack = true;
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
            } else if (!gamepad1.back && isPressingBack) {
                isPressingBack = false;
            }

            // Start button: Toggle telemetry
            if (gamepad1.start && !isPressingStart) {
                isPressingStart = true;
                isUsingTelemetry = !isUsingTelemetry;
            } else if (!gamepad1.start && isPressingStart) {
                isPressingStart = false;
            }

            // Manual alignment logic
            if (isManualAligning) {
                limelightController.align(AutoShootController.APRILTAG_ID);
                limelightController.displayAlignmentWithInitialAngle();
                if (limelightController.hasTarget() &&
                        limelightController.getTargetError() <= AutoShootController.ALIGNMENT_THRESHOLD) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            // D-pad LEFT: Distance-based auto-shoot with automatic ramp adjustment
            if (gamepad1.dpad_left && !last_dpadLeft && !autoShootController.isAutoShooting()) {
                autoShootController.executeDistanceBasedAutoShoot();
            }
            last_dpadLeft = gamepad1.dpad_left;

            // D-pad RIGHT: Toggle manual alignment
            if (gamepad1.dpad_right && !last_dpadRight) {
                if (!isManualAligning && !autoShootController.isAutoShooting()) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }
            last_dpadRight = gamepad1.dpad_right;

            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            if (isUsingTelemetry) {

                driveController.getMotorStatus();

                telemetry.addData("Expected Left Power", "%.2f", leftPower);
                telemetry.addData("Expected Right Power", "%.2f", rightPower);

                telemetry.addData("Shooting Power", "%.2f%% (%.0f RPM)",
                        SHOOTING_POWER * 100, SHOOTING_POWER * ShooterController.SHOOTER_FULL_RPM);
                telemetry.addData("Intake Power:", INTAKE_POWER);

                telemetry.addData("Shooter Encoder Velocity(MPS):", shooterController.getShooterMPS());
                telemetry.addData("Shooter Encoder Velocity(RPM):", shooterController.getShooterRPM());
                telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
                telemetry.addData("Target RPM", "%.0f", shooterController.getTargetRPM());
                telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✅ YES" : "NO");

                telemetry.addData("Robot X (inches)", "%.2f", driveController.getX());
                telemetry.addData("Robot Y (inches)", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

                // Ramp telemetry
                telemetry.addData("Ramp Angle (degrees)", "%.1f", rampController.getAngle());
                telemetry.addData("Ramp Position", "%.2f", rampController.getPosition());

                // Enhanced telemetry showing distance-based shooting info with ramp angle
                autoShootController.addTelemetry(telemetry);

                telemetry.addLine();
                telemetry.addLine("=== CONTROLS ===");
                telemetry.addLine("A: Fast Speed Mode");
                telemetry.addLine("B: Slow Speed Mode");
                telemetry.addLine("Y: Custom Power Shooter");
                telemetry.addLine("X: Stop All");
                telemetry.addLine("D-Pad UP/DOWN: Adjust Shooter Power");
                telemetry.addLine("D-Pad LEFT: Auto-Shoot (with auto ramp)");
                telemetry.addLine("D-Pad RIGHT: Toggle Manual Alignment");
                telemetry.addLine("L/R Stick Buttons: Manual Ramp Control");
            }

            telemetry.update();
        }
    }
}