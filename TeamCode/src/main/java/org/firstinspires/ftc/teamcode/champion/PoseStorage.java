package org.firstinspires.ftc.teamcode.champion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Storage for transferring robot state from Auton to Teleop
 *
 * Stores:
 * - Robot pose (x, y, heading) from odometry
 * - Turret angle (degrees)
 *
 * Usage in Auton (at end):
 *   PoseStorage.currentPose = drive.getPose();
 *   PoseStorage.turretAngle = turret.getTurretAngle();
 *
 * Usage in Teleop (at start):
 *   drive.setPosition(PoseStorage.currentPose);
 *   turret.restoreAngle(PoseStorage.turretAngle);
 */
public class PoseStorage {

    // Robot pose (x, y, heading)
    public static Pose2d currentPose = new Pose2d(new Vector2d(0, 0), Rotation2d.fromDouble(0));

    // Turret angle in degrees (0 = starting position in auton)
    public static double turretAngle = 0.0;

    /**
     * Reset all stored values
     */
    public static void reset() {
        currentPose = new Pose2d(new Vector2d(0, 0), Rotation2d.fromDouble(0));
        turretAngle = 0.0;
    }
}