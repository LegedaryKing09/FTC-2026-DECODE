package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {

    public final GoBildaPinpointDriver driver;
    public static boolean X_ENCODER_REVERSED = true;
    public static boolean Y_ENCODER_REVERSED = false;
    private Pose2d txWorldPinpoint;//world coordinate system
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);//robot coordinate system
    private long lastUpdateTime = System.nanoTime();
    public PinpointLocalizer(HardwareMap hardwareMap, double odoInPerTick, double yOffsetIn, double xOffsetIn, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        double mmPerTick = odoInPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(yOffsetIn, xOffsetIn, DistanceUnit.INCH);

        // TODO: reverse encoder directions if needed
        GoBildaPinpointDriver.EncoderDirection initialParDirection = X_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD :
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaPinpointDriver.EncoderDirection initialPerpDirection = Y_ENCODER_REVERSED ?
                GoBildaPinpointDriver.EncoderDirection.FORWARD :
                GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            // Debug logs for Pinpoint localizer
            FlightRecorder.write("PinpointLocalizer_RawPosX", driver.getPosX(DistanceUnit.INCH));
            FlightRecorder.write("PinpointLocalizer_RawPosY", driver.getPosY(DistanceUnit.INCH));
            FlightRecorder.write("PinpointLocalizer_RawHeading", Math.toDegrees(driver.getHeading(UnnormalizedAngleUnit.RADIANS)));
            FlightRecorder.write("PinpointLocalizer_WorldVelX", worldVelocity.x);
            FlightRecorder.write("PinpointLocalizer_WorldVelY", worldVelocity.y);
            FlightRecorder.write("PinpointLocalizer_RobotVelX", robotVelocity.x);
            FlightRecorder.write("PinpointLocalizer_RobotVelY", robotVelocity.y);
            FlightRecorder.write("PinpointLocalizer_X_ENCODER_REVERSED", X_ENCODER_REVERSED);
            FlightRecorder.write("PinpointLocalizer_Y_ENCODER_REVERSED", Y_ENCODER_REVERSED);

            FlightRecorder.write("PinpointLocalizer_DeviceStatus", driver.getDeviceStatus().toString());
            // Additional debug for localization accuracy during overshoot
            FlightRecorder.write("PinpointLocalizer_UpdateRate", 1.0 / (System.nanoTime() - lastUpdateTime));
            lastUpdateTime = System.nanoTime();
            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        FlightRecorder.write("PinpointLocalizer_DeviceStatus", driver.getDeviceStatus().toString());
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}