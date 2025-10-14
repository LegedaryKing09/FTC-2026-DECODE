package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.DistanceLimelightController;

/**
 * Distance-Based Auto Shooter TeleOp
 *
 * FEATURES:
 * - Calculates distance to AprilTag using Limelight
 * - Automatically adjusts shooter RPM based on distance
 * - Smart shooting: far (>100in) = 56% power, close = 59% power
 * - One-button auto shoot with alignment
 *
 * WORKFLOW:
 * 1. Driver manually positions robot and intakes balls
 * 2. Press A button to execute auto shoot:
 *    - Calculate distance to target
 *    - Determine optimal RPM based on distance
 *    - Spin up shooter to target RPM
 *    - Auto-align to AprilTag
 *    - Shoot balls
 *
 * CONTROLS:
 * - Left Stick Y: Drive forward/backward
 * - Right Stick X: Turn
 * - A Button: Auto shoot sequence (distance-based)
 * - B Button: Manual alignment toggle
 * - Y Button: Manual intake toggle
 * - X Button: Emergency stop
 * - Right Bumper: Manual shooter toggle
 * - Left Bumper: Intake eject
 * - Back Button: Speed mode toggle
 */
@Config
@TeleOp(name = "Distance-Based Auto Shooter", group = "Competition")
public class DistanceBasedAutoShooter extends LinearOpMode {

    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    DistanceLimelightController limelightController;

    // Distance-based shooting parameters
    @Config
    public static class ShootingParams {
        public static int APRILTAG_ID = 20;
        public static double DISTANCE_THRESHOLD = 100.0;  // inches

        // Far distance shooting (>100 inches)
        public static double FAR_POWER = 0.56;  // 56% power
        public static double FAR_RPM = 2800;

        // Close distance shooting (<100 inches)
        public static double CLOSE_POWER = 0.59;  // 59% power
        public static double CLOSE_RPM = 2950;

        // Alignment parameters
        public static double ALIGNMENT_THRESHOLD = 1.5;  // degrees
        public static long ALIGNMENT_TIMEOUT = 2000;     // ms
        public static long RPM_TIMEOUT = 3000;           // ms
        public static long SHOOT_DURATION = 1000;        // ms
        public static long STABILITY_DELAY = 200;        // ms
    }

    // State tracking
    private boolean isAutoShooting = false;
    private boolean isManualAligning = false;
    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // Button tracking for debouncing
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastBack = false;

    // Performance tracking
    private int shotsCompleted = 0;
    private double lastShotTime = 0;
    private double lastShotDistance = 0;
    private double lastShotRPM = 0;
    private ElapsedTime sessionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize controllers
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);

        try {
            limelightController = new DistanceLimelightController(this, driveController);
            limelightController.setTargetTag(ShootingParams.APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }

        // Setup dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== DISTANCE-BASED AUTO SHOOTER ===");
        telemetry.addLine("1. Drive to position and intake balls");
        telemetry.addLine("2. Press A for distance-based auto shoot");
        telemetry.addLine("Far (>100in): 56% power | Close: 59% power");
        telemetry.update();

        waitForStart();
        sessionTimer.reset();

        while (opModeIsActive()) {
            // CRITICAL: Always update shooter PID and odometry
            shooterController.updatePID();
            driveController.updateOdometry();

            // Manual drive control (disabled during auto shoot)
            if (!isAutoShooting && !isManualAligning) {
                double speedMult = driveController.isFastSpeedMode() ?
                        SixWheelDriveController.FAST_SPEED_MULTIPLIER :
                        SixWheelDriveController.SLOW_SPEED_MULTIPLIER;
                double turnMult = driveController.isFastSpeedMode() ?
                        SixWheelDriveController.FAST_TURN_MULTIPLIER :
                        SixWheelDriveController.SLOW_TURN_MULTIPLIER;

                double drive = -gamepad1.left_stick_y * speedMult;
                double turn = gamepad1.right_stick_x * turnMult;
                driveController.arcadeDrive(drive, turn);
            }

            // Update manual alignment if active
            if (isManualAligning) {
                limelightController.align(ShootingParams.APRILTAG_ID);
                if (limelightController.hasTarget() &&
                        limelightController.getTargetError() <= ShootingParams.ALIGNMENT_THRESHOLD) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            // A Button - Distance-based auto shoot
            if (gamepad1.a && !lastA && !isAutoShooting) {
                executeDistanceBasedShoot();
            }
            lastA = gamepad1.a;

            // B Button - Manual alignment toggle
            if (gamepad1.b && !lastB) {
                if (!isManualAligning && !isAutoShooting) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }
            lastB = gamepad1.b;

            // X Button - Emergency stop
            if (gamepad1.x && !lastX) {
                emergencyStop();
            }
            lastX = gamepad1.x;

            // Y Button - Manual intake toggle
            if (gamepad1.y && !lastY) {
                if (!intakeOn) {
                    intakeController.intakeFull();
                    intakeOn = true;
                } else {
                    intakeController.intakeStop();
                    intakeOn = false;
                }
            }
            lastY = gamepad1.y;

            // Right Bumper - Manual shooter toggle
            if (gamepad1.right_bumper && !lastRB) {
                if (!shooterOn) {
                    shooterController.setShooterRPM(ShootingParams.FAR_RPM);
                    shooterOn = true;
                } else {
                    shooterController.shooterStop();
                    shooterOn = false;
                }
            }
            lastRB = gamepad1.right_bumper;

            // Left Bumper - Intake eject
            if (gamepad1.left_bumper && !lastLB) {
                intakeController.intakeEject();
            } else if (!gamepad1.left_bumper && lastLB) {
                intakeController.intakeStop();
            }
            lastLB = gamepad1.left_bumper;

            // Back Button - Speed mode toggle
            if (gamepad1.back && !lastBack) {
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
            }
            lastBack = gamepad1.back;

            // Display telemetry
            displayTelemetry();
            telemetry.update();
        }
    }

    /**
     * Execute distance-based auto shoot sequence
     * Calculates distance, determines RPM, aligns, and shoots
     */
    private void executeDistanceBasedShoot() {
        isAutoShooting = true;

        new Thread(() -> {
            try {
                telemetry.addLine(">>> STARTING DISTANCE-BASED SHOOT <<<");
                telemetry.update();

                // Step 1: Stop any existing movement
                driveController.stopDrive();
                Thread.sleep(100);

                // Step 2: Calculate distance to target
                double distance = limelightController.getDistanceToTarget();

                if (distance <= 0) {
                    telemetry.addLine("ERROR: Cannot see target! Aborting.");
                    telemetry.update();
                    return;
                }

                lastShotDistance = distance;

                // Step 3: Determine RPM based on distance
                double targetRPM;
                double targetPower;

                if (distance > ShootingParams.DISTANCE_THRESHOLD) {
                    // Far shot
                    targetRPM = ShootingParams.FAR_RPM;
                    targetPower = ShootingParams.FAR_POWER;
                    telemetry.addData("Shot Type", "FAR (>100in)");
                } else {
                    // Close shot
                    targetRPM = ShootingParams.CLOSE_RPM;
                    targetPower = ShootingParams.CLOSE_POWER;
                    telemetry.addData("Shot Type", "CLOSE (<100in)");
                }

                lastShotRPM = targetRPM;

                telemetry.addData("Distance", "%.1f inches", distance);
                telemetry.addData("Target RPM", "%.0f", targetRPM);
                telemetry.addData("Target Power", "%.0f%%", targetPower * 100);
                telemetry.update();

                // Step 4: Spin up shooter to target RPM
                shooterController.setShooterRPM(targetRPM);

                // Step 5: Wait for shooter to reach target RPM (with timeout)
                long rpmStartTime = System.currentTimeMillis();
                while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - rpmStartTime) < ShootingParams.RPM_TIMEOUT) {
                    Thread.sleep(10);
                }

                if (!shooterController.isAtTargetRPM()) {
                    telemetry.addLine("WARNING: RPM timeout - shooting anyway");
                    telemetry.update();
                }

                // Step 6: Align to target (with error threshold check)
                if (limelightController != null) {
                    limelightController.startAlignment();

                    long alignStartTime = System.currentTimeMillis();
                    boolean alignmentGoodEnough = false;

                    // Keep aligning until within threshold OR timeout
                    while (opModeIsActive() &&
                            (System.currentTimeMillis() - alignStartTime) < ShootingParams.ALIGNMENT_TIMEOUT) {

                        limelightController.align(ShootingParams.APRILTAG_ID);

                        // Check if we're within acceptable error threshold
                        if (limelightController.hasTarget() &&
                                limelightController.getTargetError() <= ShootingParams.ALIGNMENT_THRESHOLD) {
                            alignmentGoodEnough = true;
                            break;
                        }

                        Thread.sleep(20);
                    }

                    // Stop alignment and motors
                    limelightController.stopAlignment();
                    driveController.stopDrive();

                    // Brief stabilization delay
                    Thread.sleep(ShootingParams.STABILITY_DELAY);

                    if (!alignmentGoodEnough) {
                        telemetry.addLine("WARNING: Alignment timeout - shooting anyway");
                        telemetry.update();
                    }
                }

                // Step 7: Execute the shot!
                if (shooterController.isAtTargetRPM() ||
                        Math.abs(shooterController.getRPMError()) < 200) {

                    telemetry.addLine(">>> SHOOTING NOW <<<");
                    telemetry.update();

                    // Run transfer and intake to shoot
                    transferController.transferFull();
                    intakeController.intakeFull();
                    Thread.sleep(ShootingParams.SHOOT_DURATION);

                    // Stop transfer and intake
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotsCompleted++;
                    lastShotTime = sessionTimer.seconds();

                    telemetry.addLine(">>> SHOT COMPLETED <<<");
                    telemetry.addData("Shot #", shotsCompleted);
                    telemetry.addData("Distance", "%.1f in", lastShotDistance);
                    telemetry.addData("RPM Used", "%.0f", lastShotRPM);
                    telemetry.update();
                }

                // Keep shooter running for next shot

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (Exception e) {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            } finally {
                isAutoShooting = false;
            }
        }).start();
    }

    private void emergencyStop() {
        isAutoShooting = false;
        isManualAligning = false;
        shooterOn = false;
        intakeOn = false;

        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        if (limelightController != null) {
            limelightController.stopAlignment();
        }

        telemetry.addLine(">>> EMERGENCY STOP ACTIVATED <<<");
    }

    private void displayTelemetry() {
        telemetry.addLine("╔═══════════════════════════╗");
        telemetry.addLine("║ DISTANCE-BASED SHOOTER   ║");
        telemetry.addLine("╚═══════════════════════════╝");

        // Status
        telemetry.addLine();
        if (isAutoShooting) {
            telemetry.addLine(">>> AUTO SHOOT IN PROGRESS <<<");
        } else if (isManualAligning) {
            telemetry.addLine(">>> MANUAL ALIGNMENT ACTIVE <<<");
        }

        // Distance and shooting parameters
        telemetry.addLine();
        telemetry.addLine("── DISTANCE & TARGET ──");
        if (limelightController != null && limelightController.hasTarget()) {
            double currentDistance = limelightController.getDistanceToTarget();
            telemetry.addData("Current Distance", "%.1f inches", currentDistance);

            if (currentDistance > ShootingParams.DISTANCE_THRESHOLD) {
                telemetry.addData("Recommended", "FAR shot (%.0f%% / %.0f RPM)",
                        ShootingParams.FAR_POWER * 100, ShootingParams.FAR_RPM);
            } else {
                telemetry.addData("Recommended", "CLOSE shot (%.0f%% / %.0f RPM)",
                        ShootingParams.CLOSE_POWER * 100, ShootingParams.CLOSE_RPM);
            }
        } else {
            telemetry.addData("Target", "NOT VISIBLE");
        }

        // Shooter status
        telemetry.addLine();
        telemetry.addLine("── SHOOTER STATUS ──");
        telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
        telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
        telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✓ YES" : "NO");

        // Alignment status
        if (limelightController != null && (isManualAligning || isAutoShooting)) {
            telemetry.addLine();
            telemetry.addLine("── ALIGNMENT ──");
            telemetry.addData("Has Target", limelightController.hasTarget() ? "✓" : "✗");
            if (limelightController.hasTarget()) {
                telemetry.addData("Error", "%.2f°", limelightController.getTargetError());
                telemetry.addData("Within Threshold",
                        limelightController.getTargetError() <= ShootingParams.ALIGNMENT_THRESHOLD ? "✓ YES" : "NO");
            }
        }

        // Odometry
        telemetry.addLine();
        telemetry.addLine("── POSITION ──");
        telemetry.addData("X", "%.1f in", driveController.getX());
        telemetry.addData("Y", "%.1f in", driveController.getY());
        telemetry.addData("Heading", "%.1f°", driveController.getHeadingDegrees());
        telemetry.addData("Speed Mode", driveController.isFastSpeedMode() ? "FAST" : "SLOW");

        // Session statistics
        telemetry.addLine();
        telemetry.addLine("── SESSION STATS ──");
        telemetry.addData("Shots Completed", shotsCompleted);
        if (shotsCompleted > 0) {
            telemetry.addData("Last Shot", "%.1fs ago", sessionTimer.seconds() - lastShotTime);
            telemetry.addData("Last Distance", "%.1f in", lastShotDistance);
            telemetry.addData("Last RPM", "%.0f", lastShotRPM);
        }

        // Controls
        telemetry.addLine();
        telemetry.addLine("── CONTROLS ──");
        telemetry.addLine("A: Auto Shoot | B: Manual Align");
        telemetry.addLine("Y: Intake | LB: Eject | RB: Shooter");
        telemetry.addLine("X: Emergency Stop | Back: Speed Mode");

        // Settings reminder
        telemetry.addLine();
        telemetry.addLine("── SETTINGS (FTC Dashboard) ──");
        telemetry.addData("Distance Threshold", "%.0f in", ShootingParams.DISTANCE_THRESHOLD);
        telemetry.addData("Far Shot", "%.0f%% / %.0f RPM",
                ShootingParams.FAR_POWER * 100, ShootingParams.FAR_RPM);
        telemetry.addData("Close Shot", "%.0f%% / %.0f RPM",
                ShootingParams.CLOSE_POWER * 100, ShootingParams.CLOSE_RPM);
    }
}