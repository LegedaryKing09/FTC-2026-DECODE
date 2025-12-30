package org.firstinspires.ftc.teamcode.champion.controller;


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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.PinpointLocalizer;

import org.firstinspires.ftc.teamcode.champion.Auton.drive.Drawing;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.Localizer;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.messages.TankCommandMessage;


import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;


@Config
public final class AutoTankDrive {
    public static class Params {
        // IMU orientation on the robot
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;


        // Drive wheel physical parameters
        public double wheelRadius = 1.35; // Radius of the drive wheels in inches
        public double gearRatio = 1.0; // Gear ratio between motor and wheels
        public double ticksPerRev = 537.7; // Encoder ticks per motor revolution
        public double inPerTick = 0.00197231; // (wheelRadius * 2 * Math.PI * gearRatio) / ticksPerRev;


        public double odoWheelRadius = 0.62525;
        public double odoTicksPerRev = 2000; // Encoder ticks per odometry wheel revolution
        public double odoInPerTick = (odoWheelRadius * 2 * Math.PI) / odoTicksPerRev; // Let it calculate!


        // Track width for kinematics (distance between wheels in inches)
        public double physicalTrackWidthInches = 12.309544254810191; // 14.5; //New Track Width = Current Track Width × (Target Angle / Actual Angle)


        // Path profile parameters (velocity and acceleration limits)
        public double maxWheelVelTick = 20000; // Conservative for better control
        public double maxWheelVel = maxWheelVelTick * inPerTick; // ~40 in/sec
        public double minProfileAccel = -25; // Reduced for smoother motion
        public double maxProfileAccel = 25; // Reduced for smoother motion


        public double leftMotorScale = 1.0;
        public double rightMotorScale = 1.0;


        // Feedforward control gains for motor voltage compensation
        public double kS = 1.1923587963817335; // 0.22;
        public double kV = 0.0003904177171058639;
        public double kA = 0.00006;
        public double backwardKsMultiplier = 1;


        // Feedforward control gains for turning
//        public double turnKS = 0.22;
//        public double turnKV = 0.20;
//        public double turnKA = 0.001;


        // Turn profile parameters (angular velocity and acceleration limits)
        public double maxAngVel = Math.PI; // Maximum angular velocity in radians per second
        public double maxAngAccel = Math.PI; // Maximum angular acceleration in radians per second squared


        // Ramsete controller parameters for smooth path following
        public double ramseteZeta = 0.5;
        public double ramseteBBar = 1.5;


        // Turn controller gains (proportional and velocity feedback)
        public double turnGain = 1.85; // Proportional gain for turn error correction
        public double turnVelGain = 1; // Velocity feedback gain for turn smoothing


        // Pinpoint odometry parameters for localization
        public double pinpointXOffset = 2060.8677060804002 * odoInPerTick; // 3.2; // X offset of Pinpoint sensor from robot center in inches
        public double pinpointYOffset = 3470.040645641411 * odoInPerTick; // 7.5; // Y offset of Pinpoint sensor from robot center in inches
        public double parYTicks = pinpointYOffset / odoInPerTick; // Y position of parallel encoder in tick units
        public double perpXTicks = pinpointXOffset / odoInPerTick; // X position of perpendicular encoder in tick units
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


    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx rightBack;
    public final DcMotorEx leftBack;


    public final List<DcMotorEx> leftMotors;
    public final List<DcMotorEx> rightMotors;
    public final VoltageSensor voltageSensor;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
//    public final Telemetry telemetry;


    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);


    // Telemetry data fields - updated during trajectory following and turns
    public double ramseteZeta = 0.0;
    public double ramseteBBar = 0.0;
    public double xError = 0.0;
    public double yError = 0.0;
    public double headingErrorDeg = 0.0;
    public double commandLinVel = 0.0;
    public double commandAngVelDeg = 0.0;
    public double actualLinVel = 0.0;
    public double actualAngVelDeg = 0.0;
    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double elapsedTime = 0.0;
    public double batteryVoltage = 0.0;
    public double currentX = 0.0;
    public double currentY = 0.0;
    public double currentHeadingDeg = 0.0;
    public double targetX = 0.0;
    public double targetY = 0.0;
    public double targetHeadingDeg = 0.0;
    public double leftWheelVel = 0.0;
    public double rightWheelVel = 0.0;
    public PinpointLocalizer pinpointLocalizer;
    public final Localizer localizer;
    public LazyImu lazyImu;


    public AutoTankDrive(HardwareMap hardwareMap, Pose2d pose) {
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));


//        this.telemetry = telemetry;


        // Ensure Lynx modules are up to date
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);


        // Enable bulk caching for efficient sensor reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // Initialize odometry localizer
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, PARAMS.odoInPerTick, PARAMS.pinpointYOffset, PARAMS.pinpointXOffset, pose);


        localizer = pinpointLocalizer;


        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");


        // Group motors for batch operations
        leftMotors = Arrays.asList(leftFront, leftBack);
        rightMotors = Arrays.asList(rightFront, rightBack);


        // Configure motor directions (right side reversed for tank drive)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // Configure motor braking behavior
        for (DcMotorEx motor : leftMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        // Initialize voltage sensor for feedforward compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        // Log parameters to Flight Recorder
        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }


    public void setDrivePowers(PoseVelocity2d powers) {
        // Calculate wheel velocities from robot velocities using inverse kinematics
        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(
                PoseVelocity2dDual.constant(powers, 1));


        // Find the maximum power magnitude to normalize velocities if needed
        double maxPowerMag = 1.0;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }


        // Apply normalized powers to left motors
        double leftNormalizedPower = wheelVels.left.get(0) / maxPowerMag;
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(leftNormalizedPower);
        }


        // Apply normalized powers to right motors
        double rightNormalizedPower = wheelVels.right.get(0) / maxPowerMag;
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(rightNormalizedPower);
        }
    }


    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;


        private final double[] xPoints, yPoints;


        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;


            // Sample path points for visualization (at least 2 points, up to path length / 2)
            List<Double> displacements = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[displacements.size()];
            yPoints = new double[displacements.size()];
            for (int i = 0; i < displacements.size(); i++) {
                Pose2d point = t.path.get(displacements.get(i), 1).value();
                xPoints[i] = point.position.x;
                yPoints[i] = point.position.y;
            }
        }


        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // Calculate elapsed time since action started
            double elapsedTime;
            if (beginTs < 0) {
                beginTs = Actions.now();
                elapsedTime = 0;
            } else {
                elapsedTime = Actions.now() - beginTs;
            }


            // Check if trajectory is complete
            if (elapsedTime >= timeTrajectory.duration) {
                stopMotors();
                return false;
            }


            // Get current position on trajectory
            DualNum<Time> currentPosition = timeTrajectory.profile.get(elapsedTime);
            Pose2dDual<Arclength> targetPose = timeTrajectory.path.get(currentPosition.value(), 3);
            targetPoseWriter.write(new PoseMessage(targetPose.value()));


            // Update robot pose estimate
            updatePoseEstimate();


            // Compute velocity commands using Ramsete controller
            PoseVelocity2dDual<Time> velocityCommand = new RamseteController(
                    kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(currentPosition, targetPose, pinpointLocalizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(velocityCommand));


            // === DIRECTIONAL FEEDFORWARD WITH FRICTION COMPENSATION ===
            TankKinematics.WheelVelocities<Time> wheelVelocities = kinematics.inverse(velocityCommand);
            double batteryVoltage = voltageSensor.getVoltage();


            // Get current speed to determine direction
            double currentSpeed = wheelVelocities.left.get(0);
            double effectiveKS = PARAMS.kS;


            // Apply directional friction compensation for backward motion
            if (currentSpeed < -0.5) {  // Moving backward (threshold avoids noise)
                effectiveKS *= PARAMS.backwardKsMultiplier;
            }


            // Create feedforward with direction-adjusted kS
            MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.odoInPerTick, PARAMS.kA / PARAMS.odoInPerTick);
            leftPower = feedforward.compute(wheelVelocities.left) / batteryVoltage;
            rightPower = feedforward.compute(wheelVelocities.right) / batteryVoltage;
            tankCommandWriter.write(new TankCommandMessage(batteryVoltage, leftPower, rightPower));


            leftWheelVel = wheelVelocities.left.get(0);
            rightWheelVel = wheelVelocities.right.get(0);


            // Apply motor powers with scaling for drift correction
            setMotorPowers(leftPower, rightPower);


            // Update telemetry data
            updateTelemetryData(elapsedTime, targetPose, velocityCommand, wheelVelocities, batteryVoltage, p);


            // Draw robot visualizations
            drawVisualizations(p, targetPose);


            return true;
        }


        private void stopMotors() {
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(0);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(0);
            }
        }


        private void setMotorPowers(double leftPower, double rightPower) {
            // Apply motor power scaling for drift correction
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(leftPower * PARAMS.leftMotorScale);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(rightPower * PARAMS.rightMotorScale);
            }
        }


        private void updateTelemetryData(double elapsedTime, Pose2dDual<Arclength> targetPose,
                                         PoseVelocity2dDual<Time> velocityCommand,
                                         TankKinematics.WheelVelocities<Time> wheelVelocities,
                                         double batteryVoltage, TelemetryPacket p) {
            // Current robot state
            currentX = pinpointLocalizer.getPose().position.x;
            currentY = pinpointLocalizer.getPose().position.y;
            currentHeadingDeg = Math.toDegrees(pinpointLocalizer.getPose().heading.toDouble());


            // Target state
            targetX = targetPose.value().position.x;
            targetY = targetPose.value().position.y;
            targetHeadingDeg = Math.toDegrees(targetPose.value().heading.toDouble());


            // Command velocities
            commandLinVel = velocityCommand.linearVel.value().norm();
            commandAngVelDeg = Math.toDegrees(velocityCommand.angVel.value());


            // Controller parameters
            ramseteZeta = PARAMS.ramseteZeta;
            ramseteBBar = PARAMS.ramseteBBar;


            // Calculate tracking errors
            Pose2d poseError = targetPose.value().minusExp(pinpointLocalizer.getPose());
            xError = poseError.position.x;
            yError = poseError.position.y;
            headingErrorDeg = Math.toDegrees(poseError.heading.toDouble());


            // Add telemetry data to packet
            p.put("x", currentX);
            p.put("y", currentY);
            p.put("heading (deg)", currentHeadingDeg);
            p.put("targetX", targetX);
            p.put("targetY", targetY);
            p.put("targetHeading", targetHeadingDeg);
            p.put("commandLinVel", commandLinVel);
            p.put("commandAngVel", commandAngVelDeg);
            p.put("leftWheelVel", leftWheelVel);
            p.put("rightWheelVel", rightWheelVel);
            p.put("leftPower", leftPower);
            p.put("rightPower", rightPower);
            p.put("elapsedTime", elapsedTime);
            p.put("batteryVoltage", batteryVoltage);
            p.put("ramseteZeta", ramseteZeta);
            p.put("ramseteBBar", ramseteBBar);
            p.put("xError", xError);
            p.put("yError", yError);
            p.put("headingError (deg)", headingErrorDeg);
            p.put("maxProfileAccel", PARAMS.maxProfileAccel);
            p.put("minProfileAccel", PARAMS.minProfileAccel);
            p.put("maxWheelVel", PARAMS.maxWheelVel);
            p.put("kV", PARAMS.kV);
            p.put("kA", PARAMS.kA);
            p.put("kS", PARAMS.kS);
            p.put("backwardKsMultiplier", PARAMS.backwardKsMultiplier);
            p.put("odoInPerTick", PARAMS.odoInPerTick);
            p.put("odoWheelRadius", PARAMS.odoWheelRadius);


            // Update actual velocities for debugging
            PoseVelocity2d actualVelocity = pinpointLocalizer.update();
            actualLinVel = actualVelocity.linearVel.norm();
            actualAngVelDeg = Math.toDegrees(actualVelocity.angVel);
            p.put("actualLinVel", actualLinVel);
            p.put("actualAngVel", actualAngVelDeg);


            // Log debug data
            FlightRecorder.write("TrajectoryFollow_ErrorX", poseError.position.x);
            FlightRecorder.write("TrajectoryFollow_ErrorY", poseError.position.y);
            FlightRecorder.write("TrajectoryFollow_ErrorHeading", headingErrorDeg);
            FlightRecorder.write("TrajectoryFollow_LeftPower", leftPower);
            FlightRecorder.write("TrajectoryFollow_RightPower", rightPower);
            FlightRecorder.write("TrajectoryFollow_ActualLinVel", actualLinVel);
            FlightRecorder.write("TrajectoryFollow_ActualAngVel", actualAngVelDeg);
            FlightRecorder.write("TrajectoryFollow_Voltage", batteryVoltage);
            FlightRecorder.write("TrajectoryFollow_ProfileAccelMin", PARAMS.minProfileAccel);
            FlightRecorder.write("TrajectoryFollow_PhysicalTrackWidth", PARAMS.physicalTrackWidthInches);
            FlightRecorder.write("TrajectoryFollow_BackwardKsMultiplier", PARAMS.backwardKsMultiplier);
        }


        private void drawVisualizations(TelemetryPacket p, Pose2dDual<Arclength> targetPose) {
            Canvas canvas = p.fieldOverlay();


            // Draw pose history
            drawPoseHistory(canvas);


            // Draw target robot pose in green
            canvas.setStroke("#4CAF50");
            Drawing.drawRobot(canvas, targetPose.value());


            // Draw current robot pose in blue
            canvas.setStroke("#3F51B5");
            Drawing.drawRobot(canvas, pinpointLocalizer.getPose());


            // Draw trajectory path in green
            canvas.setStroke("#4CAF50FF");
            canvas.setStrokeWidth(1);
            canvas.strokePolyline(xPoints, yPoints);
        }


        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A"); // Semi-transparent green
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
            // Calculate elapsed time since action started
            double elapsedTime;
            if (beginTs < 0) {
                beginTs = Actions.now();
                elapsedTime = 0;
            } else {
                elapsedTime = Actions.now() - beginTs;
            }


            // Check if turn is complete
            if (elapsedTime >= turn.duration) {
                stopMotors();
                return false;
            }


            // Get target pose for current time
            Pose2dDual<Time> targetPose = turn.get(elapsedTime);
            targetPoseWriter.write(new PoseMessage(targetPose.value()));


            // Update pose estimate and get current velocity
            PoseVelocity2d currentVelocity = updatePoseEstimate();


            // Compute turn command with proportional and velocity feedback control
            PoseVelocity2dDual<Time> velocityCommand = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3), // No linear velocity for turn
                    targetPose.heading.velocity().plus(
                            PARAMS.turnGain * -pinpointLocalizer.getPose().heading.minus(targetPose.heading.value()) +
                                    PARAMS.turnVelGain * (currentVelocity.angVel - targetPose.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(velocityCommand));


            // Convert to wheel velocities and calculate feedforward powers
            TankKinematics.WheelVelocities<Time> wheelVelocities = kinematics.inverse(velocityCommand);
            double batteryVoltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.odoInPerTick, PARAMS.kA / PARAMS.odoInPerTick);
            double leftPower = feedforward.compute(wheelVelocities.left) / batteryVoltage;
            double rightPower = feedforward.compute(wheelVelocities.right) / batteryVoltage;
            tankCommandWriter.write(new TankCommandMessage(batteryVoltage, leftPower, rightPower));


            // Apply motor powers with scaling
            setMotorPowers(leftPower, rightPower);


            // Draw visualizations
            drawTurnVisualizations(p, targetPose);


            // Update telemetry
            PoseVelocity2d updatedVelocity = pinpointLocalizer.update();
            p.put("localizerLinVel", updatedVelocity.linearVel);
            p.put("localizerAngVel", Math.toDegrees(updatedVelocity.angVel));
            poseHistory.add(pinpointLocalizer.getPose());


            return true;
        }


        private void stopMotors() {
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(0);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(0);
            }
        }


        private void setMotorPowers(double leftPower, double rightPower) {
            // Apply motor power scaling for drift correction
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(leftPower * PARAMS.leftMotorScale);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(rightPower * PARAMS.rightMotorScale);
            }
        }


        private void drawTurnVisualizations(TelemetryPacket p, Pose2dDual<Time> targetPose) {
            Canvas canvas = p.fieldOverlay();


            // Draw pose history
            drawPoseHistory(canvas);


            // Draw target robot pose in green
            canvas.setStroke("#4CAF50");
            Drawing.drawRobot(canvas, targetPose.value());


            // Draw current robot pose in blue
            canvas.setStroke("#3F51B5");
            Drawing.drawRobot(canvas, pinpointLocalizer.getPose());


            // Draw turn origin point in purple
            canvas.setStroke("#7C4DFFFF");
            canvas.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }


        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A"); // Semi-transparent purple
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }


    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpointLocalizer.driver.getDeviceStatus();
    }


    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d velocity = pinpointLocalizer.update();
        poseHistory.add(pinpointLocalizer.getPose());


        // Keep pose history size manageable
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }


        // Log pose estimate for debugging
        estimatedPoseWriter.write(new PoseMessage(pinpointLocalizer.getPose()));


        return velocity;
    }


    private void drawPoseHistory(Canvas canvas) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];


        int i = 0;
        for (Pose2d pose : poseHistory) {
            xPoints[i] = pose.position.x;
            yPoints[i] = pose.position.y;
            i++;
        }


        canvas.setStrokeWidth(1);
        canvas.setStroke("#3F51B5"); // Blue color
        canvas.strokePolyline(xPoints, yPoints);
    }


    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6, // Very small epsilon for numerical stability
                        new ProfileParams(0.25, 0.1, 1e-2) // Profile parameters
                ),
                beginPose, 0.0, // Start pose and time
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
