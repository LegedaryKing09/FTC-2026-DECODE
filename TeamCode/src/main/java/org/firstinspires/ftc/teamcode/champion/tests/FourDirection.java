package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Motor Direction Testing OpMode
 *
 * Tests all 16 combinations of motor directions for a 4-motor drive.
 *
 * CONTROLS:
 *   Left Stick Y  = Drive forward/backward
 *   Right Stick X = Turn left/right
 *
 *   D-Pad UP      = Next combination (0-15)
 *   D-Pad DOWN    = Previous combination (0-15)
 *
 *   A Button      = Print current working config to copy
 *
 * COMBINATIONS (F=Forward, R=Reverse):
 *   0:  LF=F, RF=F, LB=F, RB=F
 *   1:  LF=F, RF=F, LB=F, RB=R
 *   2:  LF=F, RF=F, LB=R, RB=F
 *   3:  LF=F, RF=F, LB=R, RB=R
 *   4:  LF=F, RF=R, LB=F, RB=F
 *   5:  LF=F, RF=R, LB=F, RB=R  <-- Common config
 *   6:  LF=F, RF=R, LB=R, RB=F
 *   7:  LF=F, RF=R, LB=R, RB=R
 *   8:  LF=R, RF=F, LB=F, RB=F
 *   9:  LF=R, RF=F, LB=F, RB=R
 *   10: LF=R, RF=F, LB=R, RB=F  <-- Another common config
 *   11: LF=R, RF=F, LB=R, RB=R
 *   12: LF=R, RF=R, LB=F, RB=F
 *   13: LF=R, RF=R, LB=F, RB=R
 *   14: LF=R, RF=R, LB=R, RB=F
 *   15: LF=R, RF=R, LB=R, RB=R
 *
 * When you find the correct combination:
 *   - Pushing left stick forward = robot moves forward
 *   - Pushing right stick right = robot turns right (clockwise)
 */
@TeleOp(name = "Motor Direction Tester", group = "Test")
public class FourDirection extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private int currentCombo = 0;
    private static final int MAX_COMBOS = 16;

    // Button states for edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        // Initialize motors
        try {
            lf = hardwareMap.get(DcMotor.class, "lf");
            rf = hardwareMap.get(DcMotor.class, "rf");
            lb = hardwareMap.get(DcMotor.class, "lb");
            rb = hardwareMap.get(DcMotor.class, "rb");

            // Set all to brake mode
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Run without encoders for simple testing
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("Motors initialized successfully!");
        } catch (Exception e) {
            telemetry.addLine("ERROR: " + e.getMessage());
            telemetry.addLine("Check motor names: lf, rf, lb, rb");
        }

        telemetry.addLine();
        telemetry.addLine("=== MOTOR DIRECTION TESTER ===");
        telemetry.addLine("DPad UP/DOWN = Change combination");
        telemetry.addLine("Left Stick = Drive fwd/back");
        telemetry.addLine("Right Stick = Turn left/right");
        telemetry.addLine("A Button = Print config code");
        telemetry.addLine();
        telemetry.addLine("Find the combo where:");
        telemetry.addLine("  - Stick FWD = Robot FWD");
        telemetry.addLine("  - Stick RIGHT = Robot turns RIGHT");
        telemetry.update();

        waitForStart();

        // Apply initial combination
        applyDirectionCombo(currentCombo);

        while (opModeIsActive()) {
            // Handle D-Pad UP - next combo
            if (gamepad1.dpad_up && !lastDpadUp) {
                currentCombo = (currentCombo + 1) % MAX_COMBOS;
                applyDirectionCombo(currentCombo);
            }
            lastDpadUp = gamepad1.dpad_up;

            // Handle D-Pad DOWN - previous combo
            if (gamepad1.dpad_down && !lastDpadDown) {
                currentCombo = (currentCombo - 1 + MAX_COMBOS) % MAX_COMBOS;
                applyDirectionCombo(currentCombo);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Handle A button - print code
            if (gamepad1.a && !lastA) {
                printConfigCode();
            }
            lastA = gamepad1.a;

            // Drive control
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double turn = gamepad1.right_stick_x;   // Left/right turn

            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Normalize
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Apply power
            if (lf != null) lf.setPower(leftPower);
            if (lb != null) lb.setPower(leftPower);
            if (rf != null) rf.setPower(rightPower);
            if (rb != null) rb.setPower(rightPower);

            // Telemetry
            updateTelemetry(drive, turn, leftPower, rightPower);
        }
    }

    /**
     * Apply motor direction combination
     * Each bit represents a motor: bit0=RB, bit1=LB, bit2=RF, bit3=LF
     * 0 = FORWARD, 1 = REVERSE
     */
    private void applyDirectionCombo(int combo) {
        boolean lfReverse = (combo & 0b1000) != 0;  // bit 3
        boolean rfReverse = (combo & 0b0100) != 0;  // bit 2
        boolean lbReverse = (combo & 0b0010) != 0;  // bit 1
        boolean rbReverse = (combo & 0b0001) != 0;  // bit 0

        if (lf != null) lf.setDirection(lfReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (rf != null) rf.setDirection(rfReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (lb != null) lb.setDirection(lbReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (rb != null) rb.setDirection(rbReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Get direction string for a motor based on combo
     */
    private String getDirectionStr(int combo, int bitPosition) {
        return ((combo & (1 << bitPosition)) != 0) ? "REV" : "FWD";
    }

    /**
     * Print the Java code for the current configuration
     */
    private void printConfigCode() {
        boolean lfReverse = (currentCombo & 0b1000) != 0;
        boolean rfReverse = (currentCombo & 0b0100) != 0;
        boolean lbReverse = (currentCombo & 0b0010) != 0;
        boolean rbReverse = (currentCombo & 0b0001) != 0;

        telemetry.log().clear();
        telemetry.log().add("=== COPY THIS CODE ===");
        telemetry.log().add("lf.setDirection(DcMotorSimple.Direction." + (lfReverse ? "REVERSE" : "FORWARD") + ");");
        telemetry.log().add("rf.setDirection(DcMotorSimple.Direction." + (rfReverse ? "REVERSE" : "FORWARD") + ");");
        telemetry.log().add("lb.setDirection(DcMotorSimple.Direction." + (lbReverse ? "REVERSE" : "FORWARD") + ");");
        telemetry.log().add("rb.setDirection(DcMotorSimple.Direction." + (rbReverse ? "REVERSE" : "FORWARD") + ");");
        telemetry.log().add("======================");
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry(double drive, double turn, double leftPower, double rightPower) {
        telemetry.addLine("=== MOTOR DIRECTION TESTER ===");
        telemetry.addLine();

        // Current combination
        telemetry.addData("COMBINATION", "%d / %d", currentCombo, MAX_COMBOS - 1);
        telemetry.addLine();

        // Direction display
        telemetry.addLine("--- CURRENT DIRECTIONS ---");
        telemetry.addData("LF (left front)", getDirectionStr(currentCombo, 3));
        telemetry.addData("RF (right front)", getDirectionStr(currentCombo, 2));
        telemetry.addData("LB (left back)", getDirectionStr(currentCombo, 1));
        telemetry.addData("RB (right back)", getDirectionStr(currentCombo, 0));
        telemetry.addLine();

        // Visual diagram
        telemetry.addLine("--- ROBOT DIAGRAM ---");
        telemetry.addLine(String.format("  [%s]----[%s]  FRONT",
                getDirectionStr(currentCombo, 3), getDirectionStr(currentCombo, 2)));
        telemetry.addLine("    |      |");
        telemetry.addLine(String.format("  [%s]----[%s]  BACK",
                getDirectionStr(currentCombo, 1), getDirectionStr(currentCombo, 0)));
        telemetry.addLine();

        // Input display
        telemetry.addLine("--- INPUTS ---");
        telemetry.addData("Drive (LStick Y)", "%.2f", drive);
        telemetry.addData("Turn (RStick X)", "%.2f", turn);
        telemetry.addLine();

        // Power display
        telemetry.addLine("--- MOTOR POWERS ---");
        telemetry.addData("Left Side", "%.2f", leftPower);
        telemetry.addData("Right Side", "%.2f", rightPower);
        telemetry.addLine();

        // Instructions
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("DPad UP/DOWN = Change combo");
        telemetry.addLine("A = Print config code");
        telemetry.addLine();

        // Success criteria
        telemetry.addLine("--- CORRECT WHEN ---");
        telemetry.addLine("Stick FORWARD → Robot FORWARD");
        telemetry.addLine("Stick RIGHT → Robot turns RIGHT");

        telemetry.update();
    }
}