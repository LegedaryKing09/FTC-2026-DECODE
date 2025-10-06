package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.ArrayList;

/**
 * Pure Pursuit path following controller for tank drive robots using GoBILDA Pinpoint Odometry Computer
 */
public class PurePursuitController {

    // GoBILDA Pinpoint Odometry Computer
    private final GoBildaPinpointDriver odometryComputer;

    // Localizer state
    private boolean initialized;
    private Pose2d pose;
    private Pose2d lastPose;

    // Path waypoints
    private ArrayList<Waypoint> path;
    private int currentWaypointIndex;

    // Tuning parameters
    private double lookaheadDistance;
    private double minLookahead;
    private double maxLookahead;
    private double baseSpeed;
    private double turnGain;
    private double waypointThreshold;

    /**
     * Creates a new Pure Pursuit controller with GoBILDA Pinpoint Odometry Computer integration
     */
    public PurePursuitController(HardwareMap hardwareMap, Pose2d initialPose) {
        // Initialize GoBILDA Pinpoint Odometry Computer
        odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.pose = initialPose;
        this.lastPose = initialPose;

        // Initialize path and tuning parameters
        this.path = new ArrayList<>();
        this.currentWaypointIndex = 1;
        this.lookaheadDistance = 12.0;
        this.minLookahead = 8.0;
        this.maxLookahead = 20.0;
        this.baseSpeed = 0.6;
        this.turnGain = 0.8;
        this.waypointThreshold = 3.0;
    }

    /**
     * Updates the localizer and returns motor powers for tank drive
     * Returns [leftPower, rightPower]
     */
    public double[] update() {
        // Update localizer first
        updateLocalizer();

        if (isPathComplete()) {
            return new double[]{0, 0};
        }

        Vector2d robotPos = pose.position;

        // Find lookahead point
        Waypoint lookahead = findLookaheadPoint(robotPos);

        if (lookahead == null) {
            currentWaypointIndex++;
            return update(); // Recursive call for next segment
        }

        // Check if reached current waypoint
        Waypoint currentTarget = path.get(currentWaypointIndex);
        double distanceToTarget = distanceTo(robotPos, currentTarget);

        if (distanceToTarget < waypointThreshold) {
            currentWaypointIndex++;
        }

        // Calculate tank powers
        return calculateTankPowers(pose, lookahead);
    }

    /**
     * Updates the localizer using GoBILDA Pinpoint Odometry Computer
     */
    private void updateLocalizer() {
        try {
            // Update the GoBILDA Pinpoint Odometry Computer
            odometryComputer.update();

            // Get position data (convert mm to inches for Roadrunner)
            double x = odometryComputer.getPosX() / 25.4; // Convert mm to inches
            double y = odometryComputer.getPosY() / 25.4; // Convert mm to inches
            double heading = odometryComputer.getHeading(); // Already in radians

            // Update pose
            pose = new Pose2d(new Vector2d(x, y), com.acmerobotics.roadrunner.Rotation2d.exp(heading));

            if (!initialized) {
                initialized = true;
                lastPose = pose;
            }

        } catch (Exception e) {
            // If communication fails, maintain last known pose
            // You may want to add logging here
            System.err.println("Failed to read from odometry computer: " + e.getMessage());
        }
    }


    /**
     * Sets tuning parameters for the controller
     */
    public void setParameters(double lookahead, double baseSpeed, double turnGain, double threshold) {
        this.lookaheadDistance = lookahead;
        this.baseSpeed = baseSpeed;
        this.turnGain = turnGain;
        this.waypointThreshold = threshold;
    }

    /**
     * Sets the path for the robot to follow
     */
    public void setPath(ArrayList<Waypoint> path) {
        this.path = new ArrayList<>(path);
        this.currentWaypointIndex = 1; // Start from second waypoint
    }

    /**
     * Adds a waypoint to the path
     */
    public void addWaypoint(double x, double y) {
        path.add(new Waypoint(x, y));
    }

    /**
     * Clears the current path
     */
    public void clearPath() {
        path.clear();
        currentWaypointIndex = 1;
    }

    /**
     * Returns true if the robot has completed the path
     */
    public boolean isPathComplete() {
        return currentWaypointIndex >= path.size();
    }

    /**
     * Gets the current waypoint index
     */
    public int getCurrentWaypointIndex() {
        return currentWaypointIndex;
    }

    /**
     * Gets total number of waypoints
     */
    public int getPathSize() {
        return path.size();
    }

    /**
     * Gets the current target waypoint
     */
    public Waypoint getCurrentTarget() {
        if (currentWaypointIndex < path.size()) {
            return path.get(currentWaypointIndex);
        }
        return null;
    }

    /**
     * Gets the current robot pose
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Sets the current robot pose (useful for teleop start positions)
     */
    public void setPose(Pose2d newPose) {
        this.pose = newPose;
    }

    /**
     * Gets the current lookahead point (useful for telemetry)
     */
    public Waypoint getLookaheadPoint() {
        return findLookaheadPoint(pose.position);
    }

    /**
     * Gets distance to current target waypoint
     */
    public double getDistanceToTarget() {
        if (currentWaypointIndex < path.size()) {
            return distanceTo(pose.position, path.get(currentWaypointIndex));
        }
        return 0;
    }

    private Waypoint findLookaheadPoint(Vector2d robotPos) {
        Waypoint bestPoint = null;
        double bestDistance = Double.MAX_VALUE;

        for (int i = currentWaypointIndex; i < path.size(); i++) {
            Waypoint start = path.get(i - 1);
            Waypoint end = path.get(i);

            Waypoint intersection = lineCircleIntersection(
                    start, end, robotPos.x, robotPos.y, lookaheadDistance
            );

            if (intersection != null) {
                double distToRobot = distanceTo(robotPos, intersection);
                double distDiff = Math.abs(distToRobot - lookaheadDistance);

                if (distDiff < bestDistance) {
                    bestPoint = intersection;
                    bestDistance = distDiff;
                }
            }
        }

        if (bestPoint == null && currentWaypointIndex < path.size()) {
            return path.get(currentWaypointIndex);
        }

        return bestPoint;
    }

    private Waypoint lineCircleIntersection(Waypoint start, Waypoint end,
                                            double cx, double cy, double r) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double fx = start.x - cx;
        double fy = start.y - cy;

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0 || a < 0.0001) {
            return null;
        }

        discriminant = Math.sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        double t = -1;
        if (t2 >= 0 && t2 <= 1) {
            t = t2;
        } else if (t1 >= 0 && t1 <= 1) {
            t = t1;
        }

        if (t < 0) return null;

        return new Waypoint(start.x + t * dx, start.y + t * dy);
    }

    private double[] calculateTankPowers(Pose2d currentPose, Waypoint target) {
        double dx = target.x - currentPose.position.x;
        double dy = target.y - currentPose.position.y;

        double angleToTarget = Math.atan2(dy, dx);
        double robotHeading = currentPose.heading.toDouble();

        double headingError = angleToTarget - robotHeading;
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        double distance = Math.hypot(dx, dy);
        double turn = Math.sin(headingError) * turnGain;

        double forwardSpeed = baseSpeed;

        // Slow down for sharp turns
        if (Math.abs(headingError) > Math.toRadians(45)) {
            forwardSpeed *= 0.5;
        }

        // Slow down near target
        if (distance < 12.0) {
            forwardSpeed *= Math.max(0.3, distance / 12.0);
        }

        double leftPower = forwardSpeed - turn;
        double rightPower = forwardSpeed + turn;

        // Normalize
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        return new double[]{leftPower, rightPower};
    }

    private double distanceTo(Vector2d from, Waypoint to) {
        return Math.hypot(to.x - from.x, to.y - from.y);
    }

    /**
     * Simple waypoint class representing a 2D point
     */
    public static class Waypoint {
        public double x, y;

        public Waypoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
