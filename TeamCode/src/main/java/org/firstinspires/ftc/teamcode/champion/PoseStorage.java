package org.firstinspires.ftc.teamcode.champion;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Simple static pose storage for transferring pose between Auton and TeleOp.
 * Based on Road Runner tutorial: https://learnroadrunner.com/advanced.html
 */
public class PoseStorage {
    // Static keyword lets us share data between opmodes
    public static Pose2d currentPose = new Pose2d(0, 0, 0);
}