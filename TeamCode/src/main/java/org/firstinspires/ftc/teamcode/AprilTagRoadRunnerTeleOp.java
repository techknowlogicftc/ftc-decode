package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="AprilTagRoadRunnerTeleOp", group="Navigation")
@Disabled
public class AprilTagRoadRunnerTeleOp extends LinearOpMode {

    private static final double METERS_TO_INCHES = 39.3701;
    private static final int TARGET_APRILTAG_ID = 20;
    private static final double TARGET_DISTANCE_FEET = 5.0; // 5 feet in front of tag
    private static final double TARGET_DISTANCE_INCHES = TARGET_DISTANCE_FEET * 12.0;
    
    // State variables to prevent multiple trajectory executions
    private boolean trajectoryExecuted = false;
    private boolean lastButtonState = false;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize limelight pipeline for AprilTag detection
        limelight.pipelineSwitch(0); // Use pipeline 0 for AprilTag detection
        telemetry.addData("Pipeline set to", "0 (AprilTag detection)");
        telemetry.update();

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Start limelight
        limelight.start();
        telemetry.addData("Limelight started", "Ready for AprilTag detection");
        telemetry.update();

        // Give limelight time to initialize
        sleep(1000);
        
        // Ensure pipeline is set after starting
        limelight.pipelineSwitch(0);
        telemetry.addData("Pipeline confirmed", "0");
        telemetry.update();
        
        sleep(500); // Additional time for pipeline to activate
        telemetry.addData("Limelight initialization", "Complete");
        telemetry.update();

        while (opModeIsActive()) {
            // Check for button press to start navigation
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                // Button just pressed - start navigation
                trajectoryExecuted = false;
                telemetry.addData("*** Button Pressed - Starting Navigation ***", "");
                
                // Execute navigation immediately on button press
                LLResult llResult = limelight.getLatestResult();
                
                // Clear telemetry for fresh display
                telemetry.clear();
                telemetry.addData("=== AprilTag RoadRunner Detection ===", "");
                telemetry.addData("Looking for AprilTag ID", TARGET_APRILTAG_ID);
                telemetry.addData("Trajectory Executed", trajectoryExecuted);
                
                if (llResult != null) {
                    telemetry.addData("Limelight Result", "Received");
                    telemetry.addData("Result Valid", llResult.isValid());
                    telemetry.addData("TX", String.format("%.2f", llResult.getTx()));
                    telemetry.addData("TY", String.format("%.2f", llResult.getTy()));
                    telemetry.addData("TA", String.format("%.2f", llResult.getTa()));
                    
                    if (llResult.isValid()) {
                        // Check for target AprilTag ID 20 (same logic as AprilTagTeleOpNavigation)
                        boolean targetTagFound = false;
                        if (llResult.getFiducialResults() != null) {
                            telemetry.addData("Fiducials Detected", llResult.getFiducialResults().size());
                            
                            // Display all detected fiducials
                            for (int i = 0; i < llResult.getFiducialResults().size(); i++) {
                                com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial = llResult.getFiducialResults().get(i);
                                telemetry.addData("Fiducial " + i, String.format("ID: %d, X: %.2f, Y: %.2f", 
                                    fiducial.getFiducialId(), fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees()));
                            }
                            
                            // Look for our target AprilTag ID
                            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                                if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                                    targetTagFound = true;
                                    telemetry.addData("*** TARGET TAG FOUND ***", "ID: " + TARGET_APRILTAG_ID);
                                    break;
                                }
                            }
                        } else {
                            telemetry.addData("Fiducials Detected", "0 (null result)");
                        }
                    
                    if (targetTagFound) {
                        // Get robot pose using MT2 (only after confirming target tag is detected)
                        Pose3D botPose = llResult.getBotpose_MT2();
                        
                        if (botPose != null) {
                            telemetry.addData("*** POSE DETECTED ***", "");
                            telemetry.addData("Robot X", String.format("%.1f inches", botPose.getPosition().x));
                            telemetry.addData("Robot Y", String.format("%.1f inches", botPose.getPosition().y));
                            telemetry.addData("Robot Z", String.format("%.1f inches", botPose.getPosition().z));
                            telemetry.addData("Robot Yaw", String.format("%.1f degrees", botPose.getOrientation().getYaw()));
                            
                            Pose2d limelightPose = new Pose2d(
                                botPose.getPosition().x,
                                botPose.getPosition().y,
                                Math.toRadians(botPose.getOrientation().getYaw())
                            );
                            drive.setPoseEstimate(limelightPose);
                            telemetry.addData("LL Pose", limelightPose);

                            // Calculate target pose 5 feet in front of the AprilTag
                            Pose2d targetPose = calculateTargetPoseInFrontOfTag(botPose);
                            telemetry.addData("Target Pose", String.format("X: %.1f, Y: %.1f, Heading: %.1f°", 
                                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading())));
                            telemetry.addData("Target Distance", String.format("%.1f feet from tag", TARGET_DISTANCE_FEET));

                            // Only execute trajectory if not already executed
                            if (!trajectoryExecuted) {
                                // Build and follow trajectory to position in front of AprilTag
                                telemetry.addData("*** EXECUTING TRAJECTORY ***", "Starting navigation");
                                telemetry.update();
                                
                                // Mark trajectory as executed to prevent multiple executions
                                trajectoryExecuted = true;
                                
                                // Execute the trajectory (this is a blocking call)
                                drive.followTrajectory(
                                    drive.trajectoryBuilder(limelightPose)
                                        .lineToLinearHeading(targetPose)
                                        .build()
                                );
                                
                                telemetry.addData("*** TRAJECTORY COMPLETE ***", "Navigation finished");
                                telemetry.update();
                            } else {
                                telemetry.addData("Trajectory Status", "Already executed - press A again to reset");
                            }
                        } else {
                            telemetry.addData("No valid pose", "Tag " + TARGET_APRILTAG_ID + " detected but no pose");
                        }
                    } else {
                        telemetry.addData("Limelight Result", "Invalid result");
                        telemetry.addData("Troubleshooting", "Check camera connection, lighting, and AprilTag visibility");
                        telemetry.addData("Pipeline", "Should be 0 for AprilTag detection");
                    }
                } else {
                    telemetry.addData("Limelight Result", "NULL");
                    telemetry.addData("Troubleshooting", "Check limelight connection and initialization");
                    telemetry.addData("Pipeline", "Should be 0 for AprilTag detection");
                }
                telemetry.update();
            }
            } else {
                // Button not pressed - show default telemetry
                telemetry.clear();
                telemetry.addData("=== AprilTag RoadRunner TeleOp ===", "");
                telemetry.addData("Status", trajectoryExecuted ? "Trajectory completed" : "Press A to start navigation");
                telemetry.addData("Instructions", "Press A button to navigate to 5 feet in front of AprilTag ID " + TARGET_APRILTAG_ID);
                telemetry.update();
            }
            drive.update();
        }
    }
    
    /**
     * Calculate the target pose 5 feet in front of the AprilTag.
     * The AprilTag is assumed to be at the origin (0, 0) in the field coordinate system.
     * 
     * @param robotPose The robot's current pose from limelight
     * @return The target pose 5 feet in front of the AprilTag
     */
    private Pose2d calculateTargetPoseInFrontOfTag(Pose3D robotPose) {
        // AprilTag is at origin (0, 0) in field coordinates
        double tagX = 0.0;
        double tagY = 0.0;
        
        // Calculate the direction from the tag to the robot
        double robotX = robotPose.getPosition().x;
        double robotY = robotPose.getPosition().y;
        
        // Vector from tag to robot
        double dx = robotX - tagX;
        double dy = robotY - tagY;
        
        // Distance from tag to robot
        double distanceToRobot = Math.sqrt(dx * dx + dy * dy);
        
        // Unit vector from tag to robot
        double unitX = dx / distanceToRobot;
        double unitY = dy / distanceToRobot;
        
        // Calculate target position: tag position + (unit vector * target distance)
        // This puts us 5 feet away from the tag in the same direction as the robot
        double targetX = tagX + (unitX * TARGET_DISTANCE_INCHES);
        double targetY = tagY + (unitY * TARGET_DISTANCE_INCHES);
        
        // Calculate target heading: robot should face the tag
        // Heading is the angle from the robot to the tag
        double targetHeading = Math.atan2(tagY - targetY, tagX - targetX);
        
        return new Pose2d(targetX, targetY, targetHeading);
    }
}