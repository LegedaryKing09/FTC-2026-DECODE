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
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.Drawing;
import org.firstinspires.ftc.teamcode.champion.Auton.drive.Localizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class AutoTankDrive {
    /* Fix y offset
    check trackwidth
     */

    public static class Params {
        public double inPerTick = 0.0019041574103; //  0.0019588638589618022

        // Track width for kinematics (distance between wheels in inches)
        public double physicalTrackWidthInches = 12.367583262898228; // 14.5; //New Track Width = Current Track Width × (Target Angle / Actual Angle)


        // Path profile parameters (velocity and acceleration limits)
        public double maxWheelVel = 110;
        public double minProfileAccel = -48; // Reduced for smoother motion
        public double maxProfileAccel = 48; // Reduced for smoother motion

        // Feedforward control gains for motor voltage compensation
        public double kS = 1.3289163351364959; // 0.22;
        public double kV = 0.0003243468507623318;
        public double kA = 0.00003;

        // Turn profile parameters (angular velocity and acceleration limits)
        public double maxAngVel = 2.641592653589793; // Math.PI
        public double maxAngAccel = 2.641592653589793; // Math.PI


        // Ramsete controller parameters for smooth path following
        public double ramseteZeta = 0.7;
        public double ramseteBBar = 2.0;

        // Turn controller gains (proportional and velocity feedback)
        public double turnGain = 23; // Proportional gain for turn error correction
        public double turnVelGain = 0; // Velocity feedback gain for turn smoothing

        // Pinpoint odometry parameters for localization
        public double pinpointXOffset = 3418.7735965250777 * inPerTick;  // X offset of Pinpoint sensor from robot center in inches
        public double pinpointYOffset = 2032.035531016167 * inPerTick;   // Y offset of Pinpoint sensor from robot center in inches
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


    // Telemetry data fields - updated during trajectory following and turns
    public double actualLinVel = 0.0;
    public double actualAngVelDeg = 0.0;
    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double elapsedTime = 0.0;
    public double batteryVoltage = 0.0;
    public double leftWheelVel = 0.0;
    public double rightWheelVel = 0.0;
    public double leftWheelAcc = 0.0;
    public double rightWheelAcc = 0.0;
    public PinpointLocalizer pinpointLocalizer;
    public final Localizer localizer;


    public AutoTankDrive(HardwareMap hardwareMap, Pose2d pose) {
//        this.telemetry = telemetry;


        // Ensure Lynx modules are up to date
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);


        // Enable bulk caching for efficient sensor reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // Initialize odometry localizer
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, PARAMS.pinpointYOffset, PARAMS.pinpointXOffset, pose);


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
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        // Configure motor braking behavior
        for (DcMotorEx motor : leftMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // Initialize voltage sensor for feedforward compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    public void setDrivePowers(PoseVelocity2d powers) {
        // Calculate wheel velocities from robot velocities using inverse kinematics
        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(
                PoseVelocity2dDual.constant(powers, 1));


        // Find the maximum power magnitude to normalize velocities if needed
        double maxPowerMag = 1.0;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, Math.abs(power.value()));
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

            // Update pose and get current velocity
            PoseVelocity2d actualVel = updatePoseEstimate();
            double actualLinVel = actualVel.linearVel.norm();


            // Check if trajectory is complete
            if (elapsedTime >= timeTrajectory.duration) {
                stopMotors();
                return false;
            }

            // Get current position on trajectory
            DualNum<Time> currentPosition = timeTrajectory.profile.get(elapsedTime);
            Pose2dDual<Arclength> targetPose = timeTrajectory.path.get(currentPosition.value(), 3);

            // Compute velocity commands using Ramsete controller
            PoseVelocity2dDual<Time> velocityCommand = new RamseteController(
                    kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(currentPosition, targetPose, pinpointLocalizer.getPose());

            // kinematics and feedforward
            TankKinematics.WheelVelocities<Time> wheelVelocities = kinematics.inverse(velocityCommand);
            double batteryVoltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftPower = feedforward.compute(wheelVelocities.left) / batteryVoltage;
            rightPower = feedforward.compute(wheelVelocities.right) / batteryVoltage;

            leftWheelVel = wheelVelocities.left.get(0);
            rightWheelVel = wheelVelocities.right.get(0);
            leftWheelAcc = wheelVelocities.left.get(1);
            rightWheelAcc = wheelVelocities.right.get(1);

            setMotorPowers(leftPower, rightPower);

            // Update telemetry data
            updateTelemetryData(elapsedTime, targetPose, velocityCommand, wheelVelocities, batteryVoltage, actualVel, p);

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
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(leftPower);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(rightPower);
            }
        }


        private void updateTelemetryData(double elapsedTime, Pose2dDual<Arclength> targetPose,
                                         PoseVelocity2dDual<Time> velocityCommand,
                                         TankKinematics.WheelVelocities<Time> wheelVelocities,
                                         double batteryVoltage, PoseVelocity2d actualVel, TelemetryPacket p) {
            // Current robot state
            double currentX = pinpointLocalizer.getPose().position.x;
            double currentY = pinpointLocalizer.getPose().position.y;
            double currentHeadingDeg = Math.toDegrees(pinpointLocalizer.getPose().heading.toDouble());


            // Target state
            double targetX = targetPose.value().position.x;
            double targetY = targetPose.value().position.y;
            double targetHeadingDeg = Math.toDegrees(targetPose.value().heading.toDouble());


            // Command velocities
            double commandLinVel = velocityCommand.linearVel.value().norm();
            double commandAngVelDeg = Math.toDegrees(velocityCommand.angVel.value());
            double commandLinAcc = velocityCommand.linearVel.drop(1).value().norm();


            // Calculate tracking errors
            Pose2d poseError = targetPose.value().minusExp(pinpointLocalizer.getPose());
            double xError = poseError.position.x;
            double yError = poseError.position.y;
            double headingErrorDeg = Math.toDegrees(poseError.heading.toDouble());


            // Add telemetry data to packet
            p.put("x", currentX);
            p.put("y", currentY);
            p.put("heading (deg)", currentHeadingDeg);
            p.put("targetX", targetX);
            p.put("targetY", targetY);
            p.put("targetHeading", targetHeadingDeg);
            p.put("commandLinVel", commandLinVel);
            p.put("commandAngVel", commandAngVelDeg);
            p.put("commandLinAcc", commandLinAcc);
            p.put("leftWheelVel", leftWheelVel);
            p.put("rightWheelVel", rightWheelVel);
            p.put("leftWheelAcc", leftWheelAcc);
            p.put("rightWheelAcc", rightWheelAcc);
            p.put("leftPower", leftPower);
            p.put("rightPower", rightPower);
            p.put("elapsedTime", elapsedTime);
            p.put("batteryVoltage", batteryVoltage);
            p.put("xError", xError);
            p.put("yError", yError);
            p.put("headingError (deg)", headingErrorDeg);
            p.put("maxProfileAccel", PARAMS.maxProfileAccel);
            p.put("minProfileAccel", PARAMS.minProfileAccel);
            p.put("maxWheelVel", PARAMS.maxWheelVel);
            p.put("kV", PARAMS.kV);
            p.put("kA", PARAMS.kA);
            p.put("kS", PARAMS.kS);
            p.put("inPerTick", PARAMS.inPerTick);
            p.put("ramseteZeta", PARAMS.ramseteZeta);
            p.put("ramseteBBar", PARAMS.ramseteBBar);
            actualLinVel = actualVel.linearVel.norm();
            actualAngVelDeg = Math.toDegrees(actualVel.angVel);

            p.put("actualLinVel", actualLinVel);
            p.put("actualAngVel", actualAngVelDeg);

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

            // Get target pose for current time
            Pose2dDual<Time> targetPose = turn.get(elapsedTime);

            // Update pose estimate and get current velocity
            PoseVelocity2d currentVelocity = updatePoseEstimate();

            // Calculate turning angles for telemetry
            double commandedTurnAngleDeg = Math.toDegrees(targetPose.heading.value().toDouble());
            double actualTurnAngleDeg = Math.toDegrees(pinpointLocalizer.getPose().heading.toDouble());

            // Properly normalize heading error using .log()
            double headingError = targetPose.heading.value().minus(pinpointLocalizer.getPose().heading);
            double turnAngleErrorDeg = Math.toDegrees(headingError);

            // Check if turn is complete
            if (elapsedTime >= turn.duration) {
                // Add final turn telemetry
                p.put("=== TURN COMPLETE ===", "");
                p.put("Final Commanded Angle (deg)", commandedTurnAngleDeg);
                p.put("Final Actual Angle (deg)", actualTurnAngleDeg);
                p.put("Final Turn Error (deg)", turnAngleErrorDeg);

                stopMotors();
                return false;
            }

            // Use properly normalized heading error
            // Compute turn command with proportional and velocity feedback control
            PoseVelocity2dDual<Time> velocityCommand = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3), // No linear velocity for turn
                    targetPose.heading.velocity().plus(
                            PARAMS.turnGain * headingError +  // Removed negative sign, using normalized error
                                    PARAMS.turnVelGain * (currentVelocity.angVel - targetPose.heading.velocity().value())
                    )
            );


            // Convert to wheel velocities and calculate feedforward powers
            TankKinematics.WheelVelocities<Time> wheelVelocities = kinematics.inverse(velocityCommand);
            double batteryVoltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVelocities.left) / batteryVoltage;
            double rightPower = feedforward.compute(wheelVelocities.right) / batteryVoltage;


            double feedforwardComponent = targetPose.heading.velocity().value();
            double proportionalComponent = PARAMS.turnGain * headingError;
            double velocityComponent = PARAMS.turnVelGain * (currentVelocity.angVel - targetPose.heading.velocity().value());

            leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            // Apply motor powers
            setMotorPowers(leftPower, rightPower);

            // Draw visualizations
            drawTurnVisualizations(p, targetPose);
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
            for (DcMotorEx motor : leftMotors) {
                motor.setPower(leftPower);
            }
            for (DcMotorEx motor : rightMotors) {
                motor.setPower(rightPower);
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
                        new ProfileParams(0.05, 0.1, 1e-2) // Profile parameters
                ),
                beginPose, 0.0, // Start pose and time
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
