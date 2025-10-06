package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Example usage of the Pure Pursuit Controller with GoBILDA Pinpoint Odometry Computer
 */
@Autonomous(name = "Pure Pursuit Example", group = "Champion")
public class PurePursuitExample extends LinearOpMode {
    
    private PurePursuitController controller;
    private DcMotorEx leftMotor, rightMotor;
    
    // Robot configuration - adjust these values for your robot
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0); // Starting position
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotorEx.class, "lf");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rf");
        
        // Initialize Pure Pursuit controller with GoBILDA Pinpoint Odometry Computer
        controller = new PurePursuitController(hardwareMap, START_POSE);
        
        // Configure motor directions
        leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        
        // Set up path
        setupPath();
        
        waitForStart();
        
        // Follow the path
        followPath();
    }
    
    private void setupPath() {
        // Add waypoints for your autonomous path
        controller.addWaypoint(0, 0);      // Start position
        controller.addWaypoint(24, 0);     // Move forward 24 inches
        controller.addWaypoint(24, 24);    // Turn right and move 24 inches
        controller.addWaypoint(0, 24);     // Turn left and move back 24 inches
        controller.addWaypoint(0, 0);      // Return to start
        
        // Optional: Set custom parameters
        controller.setParameters(12.0, 0.7, 0.8, 3.0);
    }
    
    private void followPath() {
        while (opModeIsActive() && !controller.isPathComplete()) {
            // Update controller and get motor powers
            double[] powers = controller.update();
            double leftPower = powers[0];
            double rightPower = powers[1];
            
            // Apply motor powers
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            
            // Optional: Add telemetry
            Pose2d currentPose = controller.getPose();
            telemetry.addData("X", currentPose.position.x);
            telemetry.addData("Y", currentPose.position.y);
            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Current Waypoint", controller.getCurrentWaypointIndex());
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
            
            sleep(20); // 50Hz update rate
        }
        
        // Stop motors when path is complete
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
