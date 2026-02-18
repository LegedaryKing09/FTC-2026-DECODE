package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * ENCODER TEST
 *
 * Push or spin the wheels manually to see encoder readings.
 * All motors are in FLOAT mode so wheels spin freely.
 *
 * GOAL: When pushing robot FORWARD, all encoders should read POSITIVE.
 *
 * Controls:
 * A = Reset all encoders to 0
 */
@Config
@TeleOp(name = "!! Encoder Test !!", group = "Test")
public class EncoderTest extends LinearOpMode {

    public static String LF_NAME = "lf";
    public static String LB_NAME = "lb";
    public static String RF_NAME = "rf";
    public static String RB_NAME = "rb";

    private DcMotorEx lf, lb, rf, rb;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        lf = hardwareMap.get(DcMotorEx.class, LF_NAME);
        lb = hardwareMap.get(DcMotorEx.class, LB_NAME);
        rf = hardwareMap.get(DcMotorEx.class, RF_NAME);
        rb = hardwareMap.get(DcMotorEx.class, RB_NAME);

        // FLOAT mode - wheels spin freely when pushed
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use encoders but don't drive
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // No power - free spinning
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        telemetry.addLine("=== ENCODER TEST ===");
        telemetry.addLine("Push robot forward to test.");
        telemetry.addLine("All values should be POSITIVE.");
        telemetry.addLine("Press A to reset encoders.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Reset encoders with A button
            if (gamepad1.a) {
                lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Get encoder positions
            int lfPos = lf.getCurrentPosition();
            int lbPos = lb.getCurrentPosition();
            int rfPos = rf.getCurrentPosition();
            int rbPos = rb.getCurrentPosition();

            // Get velocities
            double lfVel = lf.getVelocity();
            double lbVel = lb.getVelocity();
            double rfVel = rf.getVelocity();
            double rbVel = rb.getVelocity();

            // Telemetry
            telemetry.addLine("=== ENCODER TEST ===");
            telemetry.addLine("Push robot FORWARD - all should be (+)");
            telemetry.addLine("Press A to reset");
            telemetry.addLine();

            telemetry.addLine("--- POSITION (ticks) ---");
            telemetry.addData("LF (Left Front)", "%d %s", lfPos, lfPos >= 0 ? "✓" : "✗ NEGATED");
            telemetry.addData("LB (Left Back)", "%d %s", lbPos, lbPos >= 0 ? "✓" : "✗ NEGATED");
            telemetry.addData("RF (Right Front)", "%d %s", rfPos, rfPos >= 0 ? "✓" : "✗ NEGATED");
            telemetry.addData("RB (Right Back)", "%d %s", rbPos, rbPos >= 0 ? "✓" : "✗ NEGATED");
            telemetry.addLine();

            telemetry.addLine("--- VELOCITY (ticks/sec) ---");
            telemetry.addData("LF", "%.0f", lfVel);
            telemetry.addData("LB", "%.0f", lbVel);
            telemetry.addData("RF", "%.0f", rfVel);
            telemetry.addData("RB", "%.0f", rbVel);
            telemetry.addLine();

            // Check which encoders match
            telemetry.addLine("--- MATCHING PAIRS ---");
            boolean lfLbMatch = (lfPos >= 0) == (lbPos >= 0);
            boolean rfRbMatch = (rfPos >= 0) == (rbPos >= 0);
            boolean lfRbMatch = (lfPos >= 0) == (rbPos >= 0);
            boolean lbRfMatch = (lbPos >= 0) == (rfPos >= 0);

            telemetry.addData("LF & LB", lfLbMatch ? "MATCH" : "OPPOSITE");
            telemetry.addData("RF & RB", rfRbMatch ? "MATCH" : "OPPOSITE");
            telemetry.addData("LF & RB (diagonal)", lfRbMatch ? "MATCH" : "OPPOSITE");
            telemetry.addData("LB & RF (diagonal)", lbRfMatch ? "MATCH" : "OPPOSITE");

            telemetry.update();
            sleep(50);
        }
    }
}