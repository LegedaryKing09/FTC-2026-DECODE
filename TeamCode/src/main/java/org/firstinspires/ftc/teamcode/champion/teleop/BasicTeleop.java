package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.AutoShootController;
import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.RampController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@TeleOp
public class BasicTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;
    AutoShootController autoShootController;
    RampController rampController;
    public static double SHOOTING_POWER = 0.55;
    public static double INTAKE_POWER = 0;
    boolean isManualAligning = false;
    boolean lastdpadLeft = false;
    boolean lastdpadRight = false;
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
            tempLimelight = new LimelightAlignmentController(this);
            tempLimelight.setTargetTag(AutoShootController.APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }
        limelightController = tempLimelight;

        autoShootController = new AutoShootController(this, driveController, shooterController, intakeController, transferController, limelightController);

        double drive = -gamepad1.left_stick_y * SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * SixWheelDriveController.SLOW_TURN_MULTIPLIER;

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

            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                rampController.decrementAngle(2.5);
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            if (gamepad1.b && !isPressingB) {
                isPressingB = true;
                rampController.incrementAngle(2.5);
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            if (gamepad1.y && !isPressingY) {
                isPressingY = true;
                // setShooterPower already converts power (0-1) to RPM by multiplying with SHOOTER_FULL_RPM
                shooterController.setShooterPower(SHOOTING_POWER);
            } else if (!gamepad1.y && isPressingY) {
                isPressingY = false;
            }

            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                shooterController.shooterStop();
                intakeController.intakeStop();
                transferController.transferStop();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            if (gamepad1.dpad_up && SHOOTING_POWER < 1 && !isPressingDpadUp) {
                isPressingDpadUp = true;
                SHOOTING_POWER = SHOOTING_POWER + 0.01;
            } else if (!gamepad1.dpad_up && isPressingDpadUp) {
                isPressingDpadUp = false;
            }

            if (gamepad1.dpad_down && SHOOTING_POWER > 0 && !isPressingDpadDown) {
                isPressingDpadDown = true;
                SHOOTING_POWER = SHOOTING_POWER - 0.01;
            } else if (!gamepad1.dpad_down && isPressingDpadDown) {
                isPressingDpadDown = false;
            }

            if (gamepad1.right_trigger > 0.1 && !isPressingRightTrigger) {
                isPressingRightTrigger = true;
                transferController.transferFull();
            } else if (gamepad1.right_trigger < 0.1 && isPressingRightTrigger) {
                isPressingRightTrigger = false;
                transferController.transferFull();
            }

            if (gamepad1.right_bumper && !isPressingRightBumper) {
                isPressingRightBumper = true;
                intakeController.intakeFull();
            } else if (!gamepad1.right_bumper && isPressingRightBumper) {
                isPressingRightBumper = false;
                intakeController.intakeStop();
            }

            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;
                intakeController.intakeEject();
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
                intakeController.intakeStop();
            }

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

            if (gamepad1.start && !isPressingStart) {
                isPressingStart = true;
                isUsingTelemetry = !isUsingTelemetry;

            } else if (!gamepad1.start && isPressingStart) {
                isPressingStart = false;
            }

            if (isManualAligning) {
                limelightController.align(AutoShootController.APRILTAG_ID);
                if (limelightController.hasTarget() &&
                        limelightController.getTargetError() <= AutoShootController.ALIGNMENT_THRESHOLD) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            if (gamepad1.dpad_left && !lastdpadLeft && !autoShootController.isAutoShooting()) {
                autoShootController.executeAutoShootSequence(AutoShootController.TARGET_RPM);
            }
            lastdpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !lastdpadRight) {
                if (!isManualAligning && !autoShootController.isAutoShooting()) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }

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
                telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✓ YES" : "NO");
                //telemetry.addData("Is Fast Mode:", driveController.isFastSpeedMode());

                telemetry.addData("Robot X (inches)", "%.2f", driveController.getX());
                telemetry.addData("Robot Y (inches)", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

                telemetry.addData("Ramp Angle", "%.1f°", rampController.getAngle());

                autoShootController.addTelemetry(telemetry);
            }

            telemetry.update();

        }
    }
}