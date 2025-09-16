package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTag Debug Dump Test", group = "Test")
public class AprilTagDetectionTest extends LinearOpMode {

    private Limelight3A limelight;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline (20–24)

        telemetry.addLine("Ready. Press START when tag is visible.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(500); // small delay

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            telemetry.clear();
            telemetry.addLine("=== AprilTag Debug ===");

            if (result == null) {
                telemetry.addLine("Result = NULL (SDK not returning anything)");
            } else {
                telemetry.addData("isValid()", result.isValid());
                telemetry.addData("TX", result.getTx());
                telemetry.addData("TY", result.getTy());
                telemetry.addData("TA (area)", result.getTa());
                telemetry.addData("Latency ms", result.getCaptureLatency());
                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("CaptureTs", result.getTimestamp());

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addData("Fiducials detected", fiducials.size());

                for (int i = 0; i < fiducials.size(); i++) {
                    LLResultTypes.FiducialResult f = fiducials.get(i);
                    telemetry.addLine(
                            String.format("Tag %d: ID=%d, X=%.2f°, Y=%.2f°, Skew=%.2f",
                                    i,
                                    f.getFiducialId(),
                                    f.getTargetXDegrees(),
                                    f.getTargetYDegrees(),
                                    f.getSkew()
                            )
                    );
                }
            }

            telemetry.update();
            sleep(100);
        }
    }
}

