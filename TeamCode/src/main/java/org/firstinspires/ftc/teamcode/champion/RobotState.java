package org.firstinspires.ftc.teamcode.champion;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotState {
    // Static variable persists between OpModes
    private static Pose2d lastAutonPose = new Pose2d(0, 0, 0);
    private static boolean poseValid = false;

    public static void saveAutonPose(Pose2d pose) {
        lastAutonPose = pose;
        poseValid = true;
    }

    public static Pose2d getLastAutonPose() {
        return lastAutonPose;
    }

    public static boolean isPoseValid() {
        return poseValid;
    }

    public static void reset() {
        lastAutonPose = new Pose2d(0, 0, 0);
        poseValid = false;
    }
}