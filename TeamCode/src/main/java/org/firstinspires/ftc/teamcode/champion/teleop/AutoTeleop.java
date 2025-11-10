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

@TeleOp(name = "Enhanced TeleOp V2", group = "Competition")
public class AutoTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;

    // REMOVED @Config from SHOOTING_POWER to prevent dashboard override
    public static double RAMP_INCREMENT = 2.5;  // Manual ramp control increment

    // Track last button pressed for debugging
    private String lastButtonPressed = "NONE";
    private double requestedRPM = 2800;

    boolean isManualAligning = false;
    private long alignmentStartTime = 0;
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

    // Toggle states for intake
    boolean isIntakeRunning = false;
    boolean isIntakeEjecting = false;

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

        // Initialize ramp to starting position
        rampController.setTo0Degrees();

        waitForStart();

        // Set initial shooter RPM
        requestedRPM = 2800;
        shooterController.setShooterRPM(requestedRPM);
        lastButtonPressed = "INIT";

        driveController.setFastSpeed();

        while (opModeIsActive()) {

            shooterController.updatePID();

            // Calculate drive and turn based on current speed mode
            double drive, turn;
            if (driveController.isFastSpeedMode()) {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.FAST_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.FAST_TURN_MULTIPLIER;
            } else {
                drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;
            }

            driveController.arcadeDrive(drive, turn);

            // A button: Close distance setting
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                requestedRPM = 2800;
                shooterController.setShooterRPM(requestedRPM);
                rampController.setPosition(0.71);
                lastButtonPressed = "A (2800 RPM)";
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // B button: Far distance setting
            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                requestedRPM = 3100;
                shooterController.setShooterRPM(requestedRPM);
                rampController.setPosition(0.70);
                lastButtonPressed = "B (3100 RPM)";
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            // Y button: Medium distance setting
            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                requestedRPM = 2950;
                shooterController.setShooterRPM(requestedRPM);
                lastButtonPressed = "Y (2950 RPM)";
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            // X button: Stop all systems
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                shooterController.shooterStop();
                intakeController.intakeStop();
                transferController.transferStop();
                requestedRPM = 0;
                lastButtonPressed = "X (STOP)";
                // Reset intake toggle states
                isIntakeRunning = false;
                isIntakeEjecting = false;
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            // D-pad UP: Increase shooting RPM by 50
            if (gamepad1.dpad_up && !isPressingDpadUp) {
                isPressingDpadUp = true;
                requestedRPM = Math.min(requestedRPM + 50, ShooterController.SHOOTER_FULL_RPM);
                shooterController.setShooterRPM(requestedRPM);
                lastButtonPressed = "DPAD_UP (+" + requestedRPM + ")";
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            // D-pad DOWN: Decrease shooting RPM by 50
            if (gamepad1.dpad_down && !isPressingDpadDown) {
                isPressingDpadDown = true;
                requestedRPM = Math.max(requestedRPM - 50, 0);
                shooterController.setShooterRPM(requestedRPM);
                lastButtonPressed = "DPAD_DOWN (-" + requestedRPM + ")";
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }

            // Right trigger: Transfer full (hold to run)
            if (gamepad1.right_trigger > 0.1 && !isPressingRightTrigger) {
                isPressingRightTrigger = true;
                transferController.transferFull();
            } else if (gamepad1.right_trigger < 0.1 && isPressingRightTrigger) {
                isPressingRightTrigger = false;
                transferController.transferStop();
            }

            // Right bumper: Toggle intake forward
            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;

                if (isIntakeRunning) {
                    // If intake is running, stop it
                    intakeController.intakeStop();
                    isIntakeRunning = false;
                } else {
                    // If intake is not running, start it (and stop ejecting if it was)
                    intakeController.intakeFull();
                    isIntakeRunning = true;
                    isIntakeEjecting = false;
                }
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
            }

            // Left bumper: Toggle intake eject
            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;

                if (isIntakeEjecting) {
                    // If intake is ejecting, stop it
                    intakeController.intakeStop();
                    isIntakeEjecting = false;
                } else {
                    // If intake is not ejecting, start it (and stop forward if it was)
                    intakeController.intakeEject();
                    isIntakeEjecting = true;
                    isIntakeRunning = false;
                }
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
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
            if (gamepad1.dpad_left && !last_dpadLeft && autoShootController.isNotAutoShooting()) {
                lastButtonPressed = "DPAD_LEFT (AUTO)";
                autoShootController.executeDistanceBasedAutoShoot();
            }
            last_dpadLeft = gamepad1.dpad_left;

            // D-pad RIGHT: Toggle manual alignment
            if (gamepad1.dpad_right && !last_dpadRight) {
                if (!isManualAligning && autoShootController.isNotAutoShooting()) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                    alignmentStartTime = System.currentTimeMillis();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }

// Auto-disable alignment after 2.5 seconds
            if (isManualAligning && (System.currentTimeMillis() - alignmentStartTime) > 1500) {
                isManualAligning = false;
                limelightController.stopAlignment();
                driveController.stopDrive();
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

                // DEBUG INFO - CRITICAL
                telemetry.addLine("=== DEBUG INFO ===");
                telemetry.addData(">>> Last Button Pressed", lastButtonPressed);
                telemetry.addData(">>> Requested RPM", "%.0f", requestedRPM);
                telemetry.addData(">>> Actual Target RPM", "%.0f", shooterController.getTargetRPM());
                telemetry.addData(">>> Actual Current RPM", "%.0f", shooterController.getShooterRPM());
                telemetry.addData(">>> RPM Match?", requestedRPM == shooterController.getTargetRPM() ? "✅ YES" : "❌ NO");
                telemetry.addLine();

                // Updated intake status display
                String intakeStatus = "STOPPED";
                if (isIntakeRunning) intakeStatus = "RUNNING";
                else if (isIntakeEjecting) intakeStatus = "EJECTING";
                telemetry.addData("Intake Status", intakeStatus);

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
                telemetry.addLine("A: Close (2800 RPM) | B: Far (3100 RPM) | Y: Medium (2950 RPM)");
                telemetry.addLine("X: Stop All");
                telemetry.addLine("D-Pad UP/DOWN: Adjust RPM ±50");
                telemetry.addLine("D-Pad LEFT: Auto-Shoot | D-Pad RIGHT: Toggle Alignment");
                telemetry.addLine("Right Bumper: Toggle Intake Forward");
                telemetry.addLine("Left Bumper: Toggle Intake Eject");
                telemetry.addLine("Right Trigger: Transfer (hold)");
                telemetry.addLine("L/R Stick Buttons: Manual Ramp Control");
                telemetry.addLine("Back: Toggle Speed Mode | Start: Toggle Telemetry");
            }

            telemetry.update();
        }
    }
}