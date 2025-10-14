package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.TankLocalizerInputsMessage;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public final class AutoTankDrive {
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive wheel parameters for motor control
        public double wheelRadius = 1.89; //robot drives too far, decrease
        public double gearRatio = 1;
        public double ticksPerRev = 537.7;
        public double inPerTick = (wheelRadius * 2 * Math.PI * gearRatio) / ticksPerRev; //0.122649532434058

        //track width for ramsete, path following not odometry
        public double physicalTrackWidthInches = 6.8; // old_trackWidth * (θ_cmd / θ_meas)

        // feedforward parameters (in tick units)
        public double kS = 0.8; // jerking before starting, increase ; creeping when it should stop, decrease
        public double kV = 0.2; // moving slow, increase ; overshoot distance or turning too fast, decrease
        public double kA = 0.002; // lagging when accelerating, increase ; shaking when staring or stopping, decrease


        // path profile parameters
        public double maxWheelVel = 2000 * inPerTick; // overshoot, lower ; finish late or slow, higher 110.85299064868116
        public double minProfileAccel = -20;
        public double maxProfileAccel = 20; // aggressive or slipping, lower ; slow to start, increase


        // turn profile parameters
        public double maxAngVel = (2 * maxWheelVel) / physicalTrackWidthInches; // 20.0303030303030303
        public double maxAngAccel = (2 * maxProfileAccel) / physicalTrackWidthInches; // 50.70499858313351


        // Zeta : how smooth the correction is, Bbar: how fast and strong the correction is
        public double ramseteZeta = 0.7; // wobble or oscillate while correcting, increase ; lag, slow to catch up, decrease
        public double ramseteBBar = 2.0; // vibrating or jerking near corners, lower ; doesn't reach the target, increase

        public double turnGain = 0.01; // stop before full turn, increase, oscillate, decrease
        public double turnVelGain = 0.001; // drifts slowly at the end, increase ; jerks at the end, decrease

        //pinpoint odometry parameters for localization
        public double odoWheelRadius = 0.411;  //  newWheelRadius =  * target angle / odo angle
        public double odoTicksPerRev = 1180;   // newTicksRev =  * target distance / odo distance
        public double odoInPerTick = (odoWheelRadius * 2 * Math.PI) / odoTicksPerRev;
        public double pinpointXOffset = 6.0;
        public double pinpointYOffset = 3.0;

        // Pinpoint encoder directions (change if odometry reads backwards)
        public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

    }

    public static Params PARAMS = new Params();

    public final TankKinematics kinematics = new TankKinematics(PARAMS.physicalTrackWidthInches);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, rightFront, rightBack, leftBack;
    public final List<DcMotorEx> leftMotors, rightMotors;

//    public final LazyImu lazyImu;
    public final VoltageSensor voltageSensor;
    public final GoBildaPinpointDriver pinpoint;
    public final Localizer localizer; //main localizer interface
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    public class PinpointLocalizer implements Localizer {
        private Pose2d pose;

        public PinpointLocalizer(Pose2d initialPose) {
            this.pose = initialPose;

            // Configure Pinpoint with odometry parameters
            double mmPerTick = PARAMS.odoInPerTick * 25.4;
            pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);

            // Set pod offsets
            pinpoint.setOffsets(
                    PARAMS.pinpointXOffset * 25.4,  // Convert to mm
                    PARAMS.pinpointYOffset * 25.4,  // Convert to mm
                    DistanceUnit.MM
            );

            // Set encoder directions
            pinpoint.setEncoderDirections(PARAMS.xEncoderDirection, PARAMS.yEncoderDirection);

            // Reset Pinpoint
            pinpoint.resetPosAndIMU();
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
            Pose2D pose2d = new Pose2D(
                    DistanceUnit.INCH,
                    pose.position.x,
                    pose.position.y,
                    AngleUnit.RADIANS,
                    pose.heading.toDouble()
            );
            pinpoint.setPosition(pose2d);
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            pinpoint.update();

            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                // Update pose from Pinpoint
                pose = new Pose2d(
                        pinpoint.getPosX(DistanceUnit.INCH),
                        pinpoint.getPosY(DistanceUnit.INCH),
                        pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
                );

                // Return velocity
                return new PoseVelocity2d(
                        new Vector2d(
                                pinpoint.getVelX(DistanceUnit.INCH),
                                pinpoint.getVelY(DistanceUnit.INCH)
                        ),
                        pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
                );
            }

            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        public boolean isReady() {
            pinpoint.update();
            return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        }
    }

    public AutoTankDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        leftMotors = Arrays.asList(leftFront, leftBack);
        rightMotors = Arrays.asList(rightFront, rightBack);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize IMU
//        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
//                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer = new PinpointLocalizer(pose);

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    public void initializePinpoint(LinearOpMode opMode, Pose2d initialPose) {
        // Wait for Pinpoint to be ready
        while (!((PinpointLocalizer) localizer).isReady() && !opMode.isStopRequested()) {
            opMode.telemetry.addLine("Waiting for Pinpoint to initialize...");
            opMode.telemetry.update();
            opMode.sleep(100);
        }

        if (!opMode.isStopRequested()) {
            localizer.setPose(initialPose);
            opMode.telemetry.addLine("Pinpoint initialized!");
            opMode.telemetry.addData("Start Pose", "x=%.2f, y=%.2f, h=%.1f°",
                    initialPose.position.x,
                    initialPose.position.y,
                    Math.toDegrees(initialPose.heading.toDouble()));
            opMode.telemetry.update();
        }
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }
                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);
            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(
                    PARAMS.physicalTrackWidthInches,
                    PARAMS.ramseteZeta,
                    PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            // Use DRIVE wheel parameters for feedforward
            final MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    txWorldTarget.heading.velocity().plus(
                            PARAMS.turnGain * localizer.getPose().heading.minus(txWorldTarget.heading.value()) +
                                    PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.1, 1e-2)
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}