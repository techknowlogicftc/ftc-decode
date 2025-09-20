package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="AprilTagRoadRunnerTeleOp", group="Navigation")
public class AprilTagRoadRunnerTeleOp extends LinearOpMode {

    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                LLResult llResult = limelight.getLatestResult();
                if (llResult != null && llResult.isValid() && llResult.getBotpose_MT2() != null) {
                    Pose3D botPose = llResult.getBotpose_MT2();
                    Pose2d limelightPose = new Pose2d(
                        botPose.getPosition().x,
                        botPose.getPosition().y,
                        Math.toRadians(botPose.getOrientation().getYaw())
                    );
                    drive.setPoseEstimate(limelightPose);
                    telemetry.addData("LL Pose", limelightPose);

                    // Build and follow trajectory to launch zone
                    Pose2d targetPose = new Pose2d(60, 36, Math.toRadians(180));  // Example target
                    drive.followTrajectory(
                        drive.trajectoryBuilder(limelightPose)
                            .lineToLinearHeading(targetPose)
                            .build()
                    );
                } else {
                    telemetry.addLine("No valid AprilTag pose detected");
                }
                telemetry.update();
            }

            drive.update();
        }
    }
}