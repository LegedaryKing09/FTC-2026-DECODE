package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*; // imports all roadrunner classes

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.*;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.Localizer;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.TankLocalizerInputsMessage;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public final class TankDriveWithPinPoint {
    // Parameters for the tank drive, adjustable from dashboard
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        // IMU orientation relative to the hub (check FTC Docs for correct mounting)
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD; // adjust if IMU headings are inverted
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT; // adjust if pitch/roll readings are wrong

        // Physical drivetrain constants
        public double wheelRadius = 1.89; // in inches
        public double gearRatio = 1; // output (wheel) / input (motor)
        public double ticksPerRev = 71.27; // encoder ticks per motor revolution
        public double inPerTick = (wheelRadius * 2 * Math.PI * gearRatio) / ticksPerRev; // conversion from ticks → inches

        // Track width (distance between left & right wheels)
        public double trackWidthTicks = 12.0 / inPerTick; // track width converted into encoder ticks
        public double trackWidthInches = 12.0; // track width in inches

        // Feedforward constants (used for power calculation)
        public double kS = 0.1; // static gain
        public double kV = 0.01; // velocity gain
        public double kA = 0.002; // acceleration gain

        // Motion profiling parameters (path following)
        public double maxWheelVel = 2796 * inPerTick; // max wheel velocity in in/s (from motor rpm)
        public double minProfileAccel = -30; // minimum acceleration (deceleration)
        public double maxProfileAccel = 30; // maximum acceleration

        // Turning constraints
        public double maxAngVel = (2 * maxWheelVel) / trackWidthTicks; // maximum angular velocity (rad/s)
        public double maxAngAccel = (2 * maxProfileAccel) / trackWidthTicks; // maximum angular acceleration (rad/s²)

        // Ramsete controller tuning parameters
        public double ramseteZeta = 0.7; // damping coefficient (0 < zeta < 1)
        public double ramseteBBar = 2.0; // aggressiveness (> 0)

        // Turn controller PID-like gains
        public double turnGain = 0.01; // proportional gain for heading error
        public double turnVelGain = 0.001; // derivative gain for angular velocity error
    }

    // Global static params object
    public static Params PARAMS = new Params();

    // Tank drive kinematics (handles forward/inverse calculations)
    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    // Default motion constraints for turns and velocity
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // Motors
    public final DcMotorEx leftFront, rightFront, rightBack, leftBack;
    public final List<DcMotorEx> leftMotors, rightMotors;

    // IMU wrapper
    public final LazyImu lazyImu;

    // Localizers
    public PinpointLocalizer pinpointLocalizer; // uses Pinpoint module
    public Localizer localizer; // general interface

    // Voltage sensor for feedforward compensation
    public final VoltageSensor voltageSensor;

    // Pose history for debugging and field drawing
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    // Data writers for dashboard visualization
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    // Internal localizer implementation using motor encoders
    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs; // encoders on each side
        private Pose2d pose; // estimated robot pose
        private double lastLeftPos, lastRightPos; // previous encoder positions
        private boolean initialized; // flag to skip first delta

        public DriveLocalizer(Pose2d pose) {
            // Initialize encoders from motors
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m)); // wraps encoder to handle overflow
                    leftEncs.add(e);
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }
            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }
            // Set encoder directions (adjust if odometry is reversed)
            leftEncs.get(0).setDirection(DcMotorSimple.Direction.FORWARD);
            rightEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
            leftEncs.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
            rightEncs.get(1).setDirection(DcMotorSimple.Direction.FORWARD);
            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
            // TODO: reverse encoder directions if needed for correct forward motion

            this.pose = pose; // initial pose
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            // Calculates current velocity and updates pose based on encoder deltas
            Twist2dDual<Time> delta;

            // Average encoder positions and velocities for both sides
            List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
                leftReadings.add(p);
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
                rightReadings.add(p);
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            // Log inputs to dashboard
            FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                    new TankLocalizerInputsMessage(leftReadings, rightReadings));

            // Skip first update to initialize
            if (!initialized) {
                initialized = true;
                lastLeftPos = meanLeftPos;
                lastRightPos = meanRightPos;
                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            // Compute incremental motion using kinematics
            Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(PARAMS.inPerTick)
            ));

            // Update last positions
            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            // Update pose
            pose = pose.plus(twist.value());

            return twist.velocity().value(); // return current velocity
        }
    }

    // TankDrive constructor
    public TankDriveWithPinPoint(HardwareMap hardwareMap, Pose2d pose) {
        // Check firmware version for Lynx modules
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // Enable automatic bulk reads for performance
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        // Map motors from configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        // Group motors
        leftMotors = Arrays.asList(leftFront, leftBack);
        rightMotors = Arrays.asList(rightFront, rightBack);

        // Set directions (adjust as needed)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use encoders for control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when power = 0
        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse motor directions if needed
//        leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        // Initialize IMU
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        // Get voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize localizers
        localizer = new DriveLocalizer(pose);
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, new Pose2d(0, 0, 0));

        // Log parameters
        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    // Waits until Pinpoint is ready before setting pose
    public void initializePinpointPosition(LinearOpMode opMode, Pose2D pose2d) {
        while (!pinpointIsReady() && !opMode.isStopRequested()) {
            opMode.sleep(100);
        }
        pinpointLocalizer.setPoseEstimate(pose2d);
    }

    public boolean pinpointIsReady() {
        return pinpointLocalizer.isReady();
    }

    // Sets wheel powers from a desired chassis velocity
    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(PARAMS.trackWidthInches).inverse(
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

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
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

            // only draw when active; only one drive action should be active at a time
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
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
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
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}