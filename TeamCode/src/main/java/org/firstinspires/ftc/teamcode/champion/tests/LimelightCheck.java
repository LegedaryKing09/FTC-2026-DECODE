package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * Simple AprilTag viewer - shows all detected tags with TX and TY values
 */
@TeleOp(name = "Limelight Tag Viewer", group = "Test")
public class LimelightCheck extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addLine("Limelight initialized");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Limelight not found!");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight == null) {
                telemetry.addLine("No Limelight connected");
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials.isEmpty()) {
                    telemetry.addLine("No AprilTags detected");
                } else {
                    telemetry.addData("Tags seen", fiducials.size());
                    telemetry.addLine();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();
                        double tx = fiducial.getTargetXDegrees();
                        double ty = fiducial.getTargetYDegrees();

                        telemetry.addLine(String.format("Tag #%d  |  TX: %.2f°  |  TY: %.2f°", id, tx, ty));
                    }
                }
            } else {
                telemetry.addLine("No valid Limelight result");
            }

            telemetry.update();
        }
    }
}