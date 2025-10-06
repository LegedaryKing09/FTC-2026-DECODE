package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.ArrayList;
import java.util.List;

/**
 * TankSplineEmulator - Emulates smooth spline motion for tank drive
 * by breaking the path into small segments with gradual turns
 */
public class TankSplineEmulator {

    private final TankDrive drive;
    private final int segments;

    /**
     * Constructor
     * @param drive The TankDrive instance
     * @param segments Number of segments to break the path into (more = smoother, but slower)
     */
    public TankSplineEmulator(TankDrive drive, int segments) {
        this.drive = drive;
        this.segments = segments;
    }

    /**
     * Creates an emulated spline path from start pose to end pose
     * Uses a series of small turn-move segments to approximate a curve
     *
     * @param startPose Starting position and heading
     * @param endPosition Target position
     * @param endHeading Target heading
     * @return Action that executes the emulated spline
     */
    public Action createSplinePath(Pose2d startPose, Vector2d endPosition, double endHeading) {
        List<Action> actions = new ArrayList<>();

        // Calculate total displacement
        double totalDx = endPosition.x - startPose.position.x;
        double totalDy = endPosition.y - startPose.position.y;
        double totalDistance = Math.sqrt(totalDx * totalDx + totalDy * totalDy);

        // Calculate total heading change
        double startHeading = startPose.heading.toDouble();
        double totalHeadingChange = normalizeAngle(endHeading - startHeading);

        // Starting angle to target
        double directAngle = Math.atan2(totalDy, totalDx);

        Pose2d currentPose = startPose;

        // Create path segments
        for (int i = 0; i < segments; i++) {
            double t = (double) (i + 1) / segments;

            // Interpolate heading using cubic easing for smoothness
            double easedT = easeInOutCubic(t);
            double targetHeading = startHeading + totalHeadingChange * easedT;

            // Calculate intermediate point along straight line
            double interpX = startPose.position.x + totalDx * t;
            double interpY = startPose.position.y + totalDy * t;

            // Calculate angle from current position to intermediate point
            double dx = interpX - currentPose.position.x;
            double dy = interpY - currentPose.position.y;
            double angleToNext = Math.atan2(dy, dx);
            double segmentDistance = Math.sqrt(dx * dx + dy * dy);

            // Build action for this segment: turn slightly, then move forward
            final Pose2d segmentStart = currentPose;
            final double segmentAngle = angleToNext;
            final double segmentDist = segmentDistance;

            Action segmentAction = drive.actionBuilder(segmentStart)
                    .turnTo(segmentAngle)
                    .lineToX(segmentStart.position.x + segmentDist * Math.cos(segmentAngle))
                    .build();

            actions.add(segmentAction);

            // Update current pose estimate for next iteration
            currentPose = new Pose2d(
                    new Vector2d(interpX, interpY),
                    segmentAngle
            );
        }

        // Final turn to target heading
        final Pose2d finalPose = currentPose;
        Action finalTurn = drive.actionBuilder(finalPose)
                .turnTo(endHeading)
                .build();
        actions.add(finalTurn);

        // Return sequential action
        return new SequentialAction(actions);
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Cubic easing function for smooth interpolation
     */
    private double easeInOutCubic(double t) {
        return t < 0.5
                ? 4 * t * t * t
                : 1 - Math.pow(-2 * t + 2, 3) / 2;
    }

    /**
     * Sequential action that runs multiple actions in order
     */
    private static class SequentialAction implements Action {
        private final List<Action> actions;
        private int currentIndex = 0;

        public SequentialAction(List<Action> actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            if (currentIndex >= actions.size()) {
                return false; // All actions complete
            }

            boolean currentActionRunning = actions.get(currentIndex).run(packet);

            if (!currentActionRunning) {
                currentIndex++;
            }

            return currentIndex < actions.size();
        }

        @Override
        public void preview(com.acmerobotics.dashboard.canvas.Canvas canvas) {
            for (Action action : actions) {
                action.preview(canvas);
            }
        }
    }
}