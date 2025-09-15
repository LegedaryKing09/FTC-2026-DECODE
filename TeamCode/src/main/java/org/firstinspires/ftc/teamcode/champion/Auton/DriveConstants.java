package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    public double TrackWidth = 12.0;
    // Movement limits
    public double MAX_VEL = 30; // inches per second
    public double MAX_ACCEL = 30; // inches per second squared
    public double MAX_ANG_VEL = Math.toRadians(120); // radians per second
    public double MAX_ANG_ACCEL = Math.toRadians(120); // radians per second squared

    // with pinpoint, we don't need motor encoder constants, it handles all the odometry calculations
}
