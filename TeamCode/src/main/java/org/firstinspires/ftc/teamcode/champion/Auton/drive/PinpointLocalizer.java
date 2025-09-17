package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.champion.controller.PinpointOdometryDriver;

public class PinpointLocalizer implements Localizer {
    PinpointOdometryDriver pinpointOdometryDriver;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpointOdometryDriver = new PinpointOdometryDriver(hardwareMap);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return pinpointOdometryDriver.getPosition();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        pinpointOdometryDriver.setPosition(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return pinpointOdometryDriver.getVelocity();
    }

    @Override
    public void update() {
    }

    public boolean isReady() {
        return pinpointOdometryDriver.isReady();
    }
}
