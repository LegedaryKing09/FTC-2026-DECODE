package org.firstinspires.ftc.teamcode.champion;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Stores robot pose between auton and teleop.
 *
 * Uses both a static variable (fast, works within same app session)
 * AND SharedPreferences (persistent, survives app crashes/restarts).
 *
 * Auton calls savePose() in cleanup — writes to both.
 * Teleop calls loadPose() at startup — reads SharedPreferences,
 * so even if auton crashed mid-run, the last saved pose is available.
 */
public class PoseStorage {

    // In-memory (existing behavior — works if app doesn't restart)
    public static Pose2d currentPose = new Pose2d(new Vector2d(0, 0), 0);
    public static double turretAngle = 0;

    private static final String PREFS_NAME = "pose_storage";
    private static final String KEY_X = "field_x";
    private static final String KEY_Y = "field_y";
    private static final String KEY_HEADING = "field_heading";
    private static final String KEY_TURRET = "turret_angle";
    private static final String KEY_TIMESTAMP = "save_time";

    /**
     * Save pose to both static variable AND persistent storage.
     * Call this from auton cleanup() — also safe to call mid-auton
     * (e.g. after each shooting cycle) so the latest pose is always saved.
     */
    public static void savePose(Context context, double x, double y, double headingRadians, double turret) {
        // Update static (backward compatible)
        currentPose = new Pose2d(new Vector2d(x, y), headingRadians);
        turretAngle = turret;

        // Write to persistent storage
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit()
                .putFloat(KEY_X, (float) x)
                .putFloat(KEY_Y, (float) y)
                .putFloat(KEY_HEADING, (float) headingRadians)
                .putFloat(KEY_TURRET, (float) turret)
                .putLong(KEY_TIMESTAMP, System.currentTimeMillis())
                .apply();
    }

    /**
     * Load pose from persistent storage into static variables.
     * Call this at the start of teleop.
     * Returns true if a saved pose was found, false if using defaults.
     */
    public static boolean loadPose(Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);

        if (!prefs.contains(KEY_X)) {
            return false;  // No saved pose — use defaults
        }

        double x = prefs.getFloat(KEY_X, 0f);
        double y = prefs.getFloat(KEY_Y, 0f);
        double heading = prefs.getFloat(KEY_HEADING, 0f);
        double turret = prefs.getFloat(KEY_TURRET, 0f);

        currentPose = new Pose2d(new Vector2d(x, y), heading);
        turretAngle = turret;

        return true;
    }

    /**
     * Get how many seconds ago the pose was saved.
     * Useful for teleop to decide if the saved pose is still relevant.
     */
    public static long getSecondsSinceLastSave(Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        long savedTime = prefs.getLong(KEY_TIMESTAMP, 0);
        if (savedTime == 0) return Long.MAX_VALUE;
        return (System.currentTimeMillis() - savedTime) / 1000;
    }

    /**
     * Clear persistent storage. Call if you want to reset.
     */
    public static void clear(Context context) {
        currentPose = new Pose2d(new Vector2d(0, 0), 0);
        turretAngle = 0;
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit().clear().apply();
    }
}