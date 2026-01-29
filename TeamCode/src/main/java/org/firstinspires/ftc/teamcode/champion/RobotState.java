package org.firstinspires.ftc.teamcode.champion;

import android.content.Context;
import android.content.SharedPreferences;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotState {
    private static final String PREFS_NAME = "RobotPose";

    public static void saveAutonPose(OpMode opMode, Pose2d pose) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("x", (float) pose.position.x);
        editor.putFloat("y", (float) pose.position.y);
        editor.putFloat("heading", (float) pose.heading.toDouble());
        editor.putBoolean("valid", true);
        editor.apply();
    }

    public static Pose2d getLastAutonPose(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        float x = prefs.getFloat("x", 0);
        float y = prefs.getFloat("y", 0);
        float heading = prefs.getFloat("heading", 0);
        return new Pose2d(x, y, heading);
    }

    public static boolean isPoseValid(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getBoolean("valid", false);
    }

    public static void reset(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.clear();
        editor.apply();
    }

    // === HELPER METHODS FOR TELEOP BEHAVIOR ===

    public static double getAutonEndX(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getFloat("x", 0);
    }

    public static double getAutonEndY(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getFloat("y", 0);
    }

    public static double getAutonEndHeading(OpMode opMode) {
        SharedPreferences prefs = opMode.hardwareMap.appContext
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getFloat("heading", 0);
    }
}