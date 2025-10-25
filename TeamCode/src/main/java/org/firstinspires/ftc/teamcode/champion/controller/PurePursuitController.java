package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;
import java.util.Arrays;
import java.util.ArrayList;

public class PurePursuitController {

    private List<Vector2d> path;
    private double lookAheadDistance = 12.0; // inches
    private double maxSpeed = 0.6; // Reduced for better control with velocity control
    private double trackWidth = 15.0; // inches - match the track width used in auton
    private double minSpeed = 0.2; // Reduced minimum speed for finer control
    private double maxCurvatureLeft = 1.2; // Reduced for smoother turns
    private double maxCurvatureRight = 1.2; // Reduced for smoother turns
    private Vector2d targetPosition;

    public PurePursuitController() {
        // Default constructor
    }
    public void setPath(List<Vector2d> path) {
        this.path = path;
    }

    public void setParameters(double lookAhead, double maxSpeed, double trackWidth) {
        this.lookAheadDistance = lookAhead;
        this.maxSpeed = maxSpeed;
        this.trackWidth = trackWidth;
    }

    public void setTargetPosition(Vector2d targetPosition) {
        this.targetPosition = targetPosition;
        this.path = null; // Reset path to generate new one
    }

    public double[] update(Pose2d currentPose) {
        if (path == null && targetPosition != null) {
            generatePath(currentPose);
        }
        if (path == null || path.isEmpty()) {
            return new double[]{0, 0};
        }

        // Find closest point on path
        int closestIndex = findClosestPoint(currentPose.position);
        Vector2d closestPoint = path.get(closestIndex);

        // Find look-ahead point
        Vector2d lookAheadPoint = findLookAheadPoint(closestIndex, currentPose.position);

        // Vector from robot to look-ahead point
        Vector2d toLookAhead = lookAheadPoint.minus(currentPose.position);
        double distToLookAhead = toLookAhead.norm();

        if (distToLookAhead < 1e-6) {
            return new double[]{0, 0}; // At goal
        }

        // Curvature calculation
        double heading = currentPose.heading.toDouble();
        Vector2d robotHeadingVec = new Vector2d(Math.cos(heading), Math.sin(heading));
        double cross = robotHeadingVec.x * toLookAhead.y - robotHeadingVec.y * toLookAhead.x;
        double curvature = 2 * cross / (distToLookAhead * distToLookAhead);

        // Clamp curvature to prevent over-turning, using different limits for left/right
        double maxCurv = (curvature > 0) ? maxCurvatureLeft : maxCurvatureRight;
        curvature = Math.max(-maxCurvatureRight, Math.min(maxCurvatureLeft, curvature));

        // Speed based on distance to end with smoother deceleration
        double distToEnd = currentPose.position.minus(path.get(path.size() - 1)).norm();
        double speed;
        if (distToEnd < 12.0) {
            // Linear deceleration in final 12 inches
            speed = minSpeed + (maxSpeed - minSpeed) * (distToEnd / 12.0);
        } else {
            speed = maxSpeed;
        }
        speed = Math.max(minSpeed, Math.min(maxSpeed, speed)); // Clamp speed

        // Tank drive powers
        double leftPower = speed * (1 - curvature * trackWidth / 2);
        double rightPower = speed * (1 + curvature * trackWidth / 2);

        // Normalize
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        return new double[]{leftPower, rightPower};
    }

    private void generatePath(Pose2d currentPose) {
        Vector2d start = currentPose.position;
        Vector2d end = targetPosition;
        // Calculate vector from start to end
        Vector2d direction = end.minus(start);
        double distance = direction.norm();

        // Add intermediate points to smooth the path and reduce snaking
        List<Vector2d> waypoints = new java.util.ArrayList<>();
        waypoints.add(start);

        // Add points every 6 inches (half look ahead distance) for ultra-smooth motion
        double spacing = lookAheadDistance / 2.0; // 6 inches
        int numSegments = Math.max(1, (int) Math.ceil(distance / spacing));
        for (int i = 1; i <= numSegments; i++) {
            double fraction = (double) i / numSegments;
            Vector2d point = start.plus(direction.times(fraction));
            waypoints.add(point);
        }

        this.path = waypoints;
    }

    private int findClosestPoint(Vector2d position) {
        int closest = 0;
        double minDist = Double.MAX_VALUE;
        for (int i = 0; i < path.size(); i++) {
            double dist = path.get(i).minus(position).norm();
            if (dist < minDist) {
                minDist = dist;
                closest = i;
            }
        }
        return closest;
    }

    private Vector2d findLookAheadPoint(int startIndex, Vector2d position) {
        double remainingDistance = lookAheadDistance;

        for (int i = startIndex; i < path.size() - 1; i++) {
            Vector2d segmentStart = path.get(i);
            Vector2d segmentEnd = path.get(i + 1);
            Vector2d segment = segmentEnd.minus(segmentStart);
            double segmentLength = segment.norm();

            if (segmentLength < 1e-6) continue;

            Vector2d toStart = segmentStart.minus(position);
            double projection = -toStart.dot(segment) / (segmentLength * segmentLength);

            if (projection < 0) {
                // Closest to start
                double distToStart = toStart.norm();
                if (distToStart >= remainingDistance) {
                    return position.plus(toStart.div(distToStart).times(remainingDistance));
                } else {
                    remainingDistance -= distToStart;
                }
            } else if (projection > 1) {
                // Closest to end
                Vector2d toEnd = segmentEnd.minus(position);
                double distToEnd = toEnd.norm();
                if (distToEnd >= remainingDistance) {
                    return position.plus(toEnd.div(distToEnd).times(remainingDistance));
                } else {
                    remainingDistance -= distToEnd;
                }
            } else {
                // On segment
                Vector2d closest = segmentStart.plus(segment.times(projection));
                Vector2d toClosest = closest.minus(position);
                double distToClosest = toClosest.norm();
                if (distToClosest >= remainingDistance) {
                    return position.plus(toClosest.div(distToClosest).times(remainingDistance));
                } else {
                    remainingDistance -= distToClosest;
                }
            }
        }

        // End of path
        return path.get(path.size() - 1);
    }
}