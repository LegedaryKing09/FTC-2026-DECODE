package org.firstinspires.ftc.teamcode.champion.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightTrackingController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Config
@TeleOp
public class FirstCompositeTeleop extends LinearOpMode {

    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightTrackingController LimelightTrackingController;
    public static double SHOOTING_POWER = 0.8;
    public static double INTAKE_POWER = 0;

    boolean isUsingTelemetry = true;
    boolean isPressingB = false;
    boolean isPressingA = false;
    boolean isPressingY = false;
    boolean isPressingX = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingDpadLeft = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingStart = false;

    @Override
    public void runOpMode() {
        if (gamepad1.y && !isPressingY) {
            isPressingY = true;
            intakeController.intakeFull();
            driveController.tankDrive(0.8, 0.8);
            sleep(300);
            driveController.tankDrive(-0.8, -0.8);
            sleep(300);
            driveController.stopDrive();
            intakeController.intakeStop();
            transferController.transferStop();
            shooterController.setShooterPower(SHOOTING_POWER);
            while (!LimelightTrackingController.isAligned()) {
                LimelightTrackingController.TrackingResult result = LimelightTrackingController.alignToTarget(20);
                driveController.tankDrive(result.leftPower, result.rightPower);
            }
            driveController.stopDrive();
            sleep(2750);
            transferController.transferFull();
            intakeController.intakeFull();
            sleep(1500);
        } else if (!gamepad1.y && isPressingY) {
            isPressingY = false;
        }
    }
}