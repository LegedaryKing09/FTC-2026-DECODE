package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Full power shooter + intake/transfer/uptake test.
 *
 * HOW TO USE:
 * 1. Run opmode
 * 2. Hold A to spin both shooter motors at full power
 * 3. Release A to stop shooters
 * 4. Press B to toggle intake + transfer + uptake (feed balls)
 * 5. Hold LB to reverse intake + transfer + uptake (vomit)
 * 6. Toggle directions via FTC Dashboard or:
 *    - D-pad Up: toggle motor1 direction
 *    - D-pad Down: toggle motor2 direction
 * 7. Both motors should spin the SAME way for shooting
 *
 * Tune MOTOR1_REVERSED and MOTOR2_REVERSED until both spin correctly.
 */
@Config
@TeleOp(name = "Shooter + Feed Test", group = "Test")
public class ShooterPower extends LinearOpMode {

    public static boolean MOTOR1_REVERSED = false;
    public static boolean MOTOR2_REVERSED = true;
    public static double POWER = 1.0;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double UPTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        // Shooter motors
        DcMotorEx motor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter1");
        DcMotorEx motor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter2");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake + Transfer motors
        DcMotor intakeMotor = null;
        DcMotor transferMotor = null;
        try { intakeMotor = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception e) {}
        try { transferMotor = hardwareMap.get(DcMotor.class, "transfer"); } catch (Exception e) {}

        // Uptake servos
        CRServo uptakeServo1 = null;
        CRServo uptakeServo2 = null;
        try { uptakeServo1 = hardwareMap.get(CRServo.class, "servo1"); } catch (Exception e) {}
        try { uptakeServo2 = hardwareMap.get(CRServo.class, "servo2"); } catch (Exception e) {}

        telemetry.addData("Status", "Ready - A=shoot, B=toggle feed, LB=vomit");
        telemetry.update();

        waitForStart();

        boolean lastUp = false, lastDown = false;
        boolean lastB = false;
        boolean feedActive = false;

        while (opModeIsActive()) {
            // Toggle shooter directions
            if (gamepad1.dpad_up && !lastUp) MOTOR1_REVERSED = !MOTOR1_REVERSED;
            lastUp = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !lastDown) MOTOR2_REVERSED = !MOTOR2_REVERSED;
            lastDown = gamepad1.dpad_down;

            // A = hold to shoot
            if (gamepad1.a) {
                motor1.setPower(MOTOR1_REVERSED ? -POWER : POWER);
                motor2.setPower(MOTOR2_REVERSED ? -POWER : POWER);
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            // B = toggle intake + transfer + uptake
            if (gamepad1.b && !lastB) {
                feedActive = !feedActive;
            }
            lastB = gamepad1.b;

            // LB = hold to vomit (reverse all)
            if (gamepad1.left_bumper) {
                if (intakeMotor != null) intakeMotor.setPower(-INTAKE_POWER);
                if (transferMotor != null) transferMotor.setPower(-TRANSFER_POWER);
                if (uptakeServo1 != null) uptakeServo1.setPower(-UPTAKE_POWER);
                if (uptakeServo2 != null) uptakeServo2.setPower(-UPTAKE_POWER);
            } else if (feedActive) {
                if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
                if (transferMotor != null) transferMotor.setPower(TRANSFER_POWER);
                if (uptakeServo1 != null) uptakeServo1.setPower(UPTAKE_POWER);
                if (uptakeServo2 != null) uptakeServo2.setPower(UPTAKE_POWER);
            } else {
                if (intakeMotor != null) intakeMotor.setPower(0);
                if (transferMotor != null) transferMotor.setPower(0);
                if (uptakeServo1 != null) uptakeServo1.setPower(0);
                if (uptakeServo2 != null) uptakeServo2.setPower(0);
            }

            // RPM from motor1 encoder
            double tps = motor1.getVelocity();
            double rpm = Math.abs((tps / 28.0) * 60.0);

            telemetry.addData("A = SHOOT", gamepad1.a ? "SPINNING" : "off");
            telemetry.addData("B = FEED", feedActive ? "ON" : "OFF");
            telemetry.addData("LB = VOMIT", gamepad1.left_bumper ? "REVERSING" : "off");
            telemetry.addLine();
            telemetry.addData("Motor1 Rev", MOTOR1_REVERSED);
            telemetry.addData("Motor2 Rev", MOTOR2_REVERSED);
            telemetry.addData("Power", "%.2f", POWER);
            telemetry.addData("RPM (motor1)", "%.0f", rpm);
            telemetry.addData("DpadUp", "toggle M1 dir");
            telemetry.addData("DpadDown", "toggle M2 dir");
            telemetry.update();

            sleep(20);
        }

        motor1.setPower(0);
        motor2.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (transferMotor != null) transferMotor.setPower(0);
        if (uptakeServo1 != null) uptakeServo1.setPower(0);
        if (uptakeServo2 != null) uptakeServo2.setPower(0);
    }
}