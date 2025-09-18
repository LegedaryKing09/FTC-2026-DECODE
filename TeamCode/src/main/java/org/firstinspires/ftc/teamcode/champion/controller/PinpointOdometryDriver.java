package org.firstinspires.ftc.teamcode.champion.controller;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class PinpointOdometryDriver {
    public static String PINPOINT_NAME = "odo";
    public static double X_OFFSET = 40;
    public static double Y_OFFSET = -105;
    public static double TICKS_PER_MM = 74.88;

    private PinpointDriver pinpointDriver;

    public PinpointOdometryDriver(LinearOpMode opMode) {
        pinpointDriver = opMode.hardwareMap.get(PinpointDriver.class, PINPOINT_NAME);
        pinpointDriver.setOffsets(X_OFFSET, Y_OFFSET);
        pinpointDriver.setEncoderResolution(TICKS_PER_MM);
        pinpointDriver.setEncoderDirections(
                PinpointDriver.EncoderDirection.FORWARD,
                PinpointDriver.EncoderDirection.FORWARD
        );
        pinpointDriver.resetPosAndIMU();
    }

    public PinpointOdometryDriver(HardwareMap hardwareMap) {
        pinpointDriver = hardwareMap.get(PinpointDriver.class, PINPOINT_NAME);
        pinpointDriver.setOffsets(X_OFFSET, Y_OFFSET);
        pinpointDriver.setEncoderResolution(TICKS_PER_MM);
        pinpointDriver.setEncoderDirections(
                PinpointDriver.EncoderDirection.FORWARD,
                PinpointDriver.EncoderDirection.FORWARD
        );
        pinpointDriver.resetPosAndIMU();
    }

    /**
     * Get output from pinpoint.
     *
     * @return Position in inches and radians
     */
    public Pose2d getPosition() {
        pinpointDriver.update();
        Pose2D odometryPose = pinpointDriver.getPosition();

        Log.d("Pinpoint", "runOpMode: (" + odometryPose.getX(DistanceUnit.INCH) + ", " + odometryPose.getY(DistanceUnit.INCH) + ");");

        return new Pose2d(
                odometryPose.getX(DistanceUnit.INCH),
                odometryPose.getY(DistanceUnit.INCH),
                odometryPose.getHeading(AngleUnit.RADIANS)
        );
    }

    /**
     * Set position.
     *
     * @param pose in inches and radians
     */
    public void setPosition(Pose2d pose) {
        pinpointDriver.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        pose.getX(),
                        pose.getY(),
                        AngleUnit.RADIANS,
                        pose.getHeading()
                )
        );
    }

    public Pose2d getVelocity() {
        pinpointDriver.update();
        Pose2D velo = pinpointDriver.getVelocity();
        return new Pose2d(
                velo.getX(DistanceUnit.INCH),
                velo.getY(DistanceUnit.INCH),
                velo.getHeading(AngleUnit.RADIANS)
        );
    }

    public boolean isReady() {
        pinpointDriver.update();
        return pinpointDriver.getDeviceStatus() == PinpointDriver.DeviceStatus.READY;
    }
}
