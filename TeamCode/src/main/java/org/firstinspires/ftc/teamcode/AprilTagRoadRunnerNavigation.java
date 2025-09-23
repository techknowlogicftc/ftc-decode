package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "AprilTag RoadRunner Navigation", group = "Navigation")
public class AprilTagRoadRunnerNavigation extends LinearOpMode {

    // Hardware declarations
    private Limelight3A limelight;
    private IMU imu;
    private SampleMecanumDrive drive;
    
    // Navigation constants
    private static final int TARGET_APRILTAG_ID = 20;
    private static final double TARGET_DISTANCE_FEET = 5.0;
    private static final double TARGET_DISTANCE_INCHES = TARGET_DISTANCE_FEET * 12.0;
    
    // State variables
    private boolean lastButtonState = false;
    private boolean navigationInProgress = false;
    private boolean navigationCompleted = false;
    private int loopCounter = 0;
    private long lastValidPoseTime = 0;
    private boolean tagRecentlyDetected = false;
    private Pose3D lastValidPose = null;
    private static final long POSE_TIMEOUT_MS = 2000; // 2 seconds grace period

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("AprilTag RoadRunner Navigation", "Initializing...");
        telemetry.update();
        
        // Initialize hardware
        initializeHardware();
        
        telemetry.addData("Hardware initialized", "Ready to start");
        telemetry.addData("Instructions", "Press A button to navigate to 5 feet in front of AprilTag ID " + TARGET_APRILTAG_ID);
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Start limelight
        limelight.start();
        telemetry.addData("Limelight started", "Ready for navigation");
        telemetry.addData("IMPORTANT", "Ensure fieldmap is uploaded to Limelight!");
        telemetry.addData("Fieldmap", "Must be configured via web interface");
        telemetry.update();
        
        // Give limelight time to initialize
        sleep(1000);
        
        // Main loop
        while (opModeIsActive()) {
            loopCounter++;
            
            // Clear telemetry for fresh display
            telemetry.clear();
            telemetry.addData("=== AprilTag RoadRunner Navigation ===", "");
            telemetry.addData("Loop Counter", loopCounter);
            telemetry.addData("Navigation Status", getNavigationStatus());
            telemetry.addData("A Button Pressed", gamepad1.a);
            
            // Check for A button press to start navigation
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                // Button just pressed
                if (!navigationInProgress && !navigationCompleted) {
                    startNavigation();
                } else if (navigationCompleted) {
                    // Reset for another navigation attempt
                    resetNavigation();
                }
            }
            lastButtonState = currentButtonState;
            
            if (navigationInProgress) {
                // Perform navigation
                performNavigation();
            } else {
                // Show AprilTag detection info
                showAprilTagInfo();
            }
            
            // Update RoadRunner
            drive.update();
            
            telemetry.update();
        }
        
        // Final stop
        telemetry.addData("TeleOp Complete", "Stopped");
        telemetry.update();
    }
    
    private void initializeHardware() {
        // Initialize RoadRunner drive
        drive = new SampleMecanumDrive(hardwareMap);
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use pipeline 0 for AprilTag detection
        
        // IMPORTANT: Field space localization must be configured through Limelight web interface
        // 1. Upload the correct .fmap file to the Limelight camera
        // 2. Ensure the fieldmap accurately reflects the real-world AprilTag positions
        // 3. Configure camera offset and rotation relative to robot
        // Without proper fieldmap, getBotpose_MT2() will give inaccurate coordinates
        
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    
    private String getNavigationStatus() {
        if (navigationCompleted) {
            return "COMPLETED - Press A to reset";
        } else if (navigationInProgress) {
            return "IN PROGRESS";
        } else {
            return "READY - Press A to start";
        }
    }
    
    private void startNavigation() {
        telemetry.addData("*** STARTING NAVIGATION ***", "");
        navigationInProgress = true;
        navigationCompleted = false;
    }
    
    private void resetNavigation() {
        telemetry.addData("*** RESETTING NAVIGATION ***", "");
        navigationInProgress = false;
        navigationCompleted = false;
        tagRecentlyDetected = false;
        lastValidPose = null;
    }
    
    private void performNavigation() {
        telemetry.addData("--- Navigation in Progress ---", "");
        
        // Update robot orientation with IMU
        updateRobotOrientation();
        
        // Get limelight result
        LLResult llResult = limelight.getLatestResult();
        long currentTime = System.currentTimeMillis();
        
        if (llResult != null && llResult.isValid()) {
            // Check for target AprilTag ID 20
            boolean targetTagFound = false;
            Pose3D botPose = null;
            
            // Look for our target AprilTag ID
            if (llResult.getFiducialResults() != null) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                        targetTagFound = true;
                        telemetry.addData("*** TARGET TAG FOUND ***", "ID: " + TARGET_APRILTAG_ID);
                        break;
                    }
                }
            }
            
            if (targetTagFound) {
                // Get robot pose using MT2
                botPose = llResult.getBotpose_MT2();
                
                if (botPose != null) {
                    telemetry.addData("*** POSE DETECTED ***", "");
                    telemetry.addData("Robot X", String.format("%.1f inches", botPose.getPosition().x * 39.37));
                    telemetry.addData("Robot Y", String.format("%.1f inches", botPose.getPosition().y * 39.37));
                    telemetry.addData("Robot Z", String.format("%.1f inches", botPose.getPosition().z * 39.37));
                    telemetry.addData("Robot Yaw", String.format("%.1f degrees", botPose.getOrientation().getYaw()));
                    
                    // Update pose tracking
                    lastValidPose = botPose;
                    lastValidPoseTime = currentTime;
                    tagRecentlyDetected = true;
                    
                    // Execute RoadRunner navigation
                    executeRoadRunnerNavigation(botPose);
                } else {
                    telemetry.addData("No valid pose", "Tag 20 detected but no pose");
                    handlePoseLoss(currentTime);
                }
            } else {
                telemetry.addData("Target Tag Status", "AprilTag ID " + TARGET_APRILTAG_ID + " not in view");
                telemetry.addData("Action", "Looking for tag ID " + TARGET_APRILTAG_ID);
                handlePoseLoss(currentTime);
            }
            
            // Display basic limelight data
            telemetry.addData("Limelight Data", "");
            telemetry.addData("TX", String.format("%.2f", llResult.getTx()));
            telemetry.addData("TY", String.format("%.2f", llResult.getTy()));
            telemetry.addData("TA", String.format("%.2f", llResult.getTa()));
            
        } else {
            telemetry.addData("No valid result", "No AprilTag detected");
            handlePoseLoss(currentTime);
        }
        
        telemetry.addData("Instructions", "Navigation in progress...");
    }
    
    private void showAprilTagInfo() {
        telemetry.addData("--- AprilTag Detection Info ---", "");
        telemetry.addData("Target Tag ID", TARGET_APRILTAG_ID);
        
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            // Check if target tag is detected
            boolean targetTagFound = false;
            if (llResult.getFiducialResults() != null) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                        targetTagFound = true;
                        telemetry.addData("Target Tag " + TARGET_APRILTAG_ID, "DETECTED");
                        break;
                    }
                }
            }
            
            if (targetTagFound) {
                telemetry.addData("AprilTag Detected", "YES - Target ID " + TARGET_APRILTAG_ID);
                telemetry.addData("TX", String.format("%.2f", llResult.getTx()));
                telemetry.addData("TY", String.format("%.2f", llResult.getTy()));
                telemetry.addData("TA", String.format("%.2f", llResult.getTa()));
                
                Pose3D botPose = llResult.getBotpose_MT2();
                if (botPose != null) {
                    telemetry.addData("Robot Position", String.format("X: %.1f, Y: %.1f", 
                        botPose.getPosition().x * 39.37, botPose.getPosition().y * 39.37));
                    telemetry.addData("Distance from Tag", String.format("%.1f inches", 
                        Math.sqrt((botPose.getPosition().x * 39.37) * (botPose.getPosition().x * 39.37) + 
                                 (botPose.getPosition().y * 39.37) * (botPose.getPosition().y * 39.37))));
                }
            } else {
                telemetry.addData("AprilTag Detected", "NO - Target ID " + TARGET_APRILTAG_ID + " not found");
                if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                    telemetry.addData("Available Tag IDs", "Other tags detected but not target");
                }
            }
        } else {
            telemetry.addData("AprilTag Detected", "NO");
        }
        
        telemetry.addData("Instructions", "Press A button to start navigation to 5 feet in front of tag");
        
        // Add a simple test to verify fieldmap accuracy
        if (llResult != null && llResult.isValid() && llResult.getFiducialResults() != null) {
            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                    telemetry.addData("Fieldmap Test", "");
                    telemetry.addData("Tag ID " + TARGET_APRILTAG_ID + " detected", "YES");
                    telemetry.addData("TX (degrees)", String.format("%.2f", fiducial.getTargetXDegrees()));
                    telemetry.addData("TY (degrees)", String.format("%.2f", fiducial.getTargetYDegrees()));
                    
                    // Calculate expected distance from TX/TY values
                    double expectedDistance = Math.sqrt(
                        fiducial.getTargetXDegrees() * fiducial.getTargetXDegrees() + 
                        fiducial.getTargetYDegrees() * fiducial.getTargetYDegrees()
                    );
                    telemetry.addData("Expected distance", String.format("%.1f degrees", expectedDistance));
                    break;
                }
            }
        }
    }
    
    private void updateRobotOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        
        telemetry.addData("IMU Data", "");
        telemetry.addData("Yaw", String.format("%.1f degrees", orientation.getYaw()));
        telemetry.addData("Pitch", String.format("%.1f degrees", orientation.getPitch()));
        telemetry.addData("Roll", String.format("%.1f degrees", orientation.getRoll()));
    }
    
    private void executeRoadRunnerNavigation(Pose3D botPose) {
        telemetry.addData("--- RoadRunner Navigation ---", "");
        
        // Convert limelight pose to RoadRunner pose (meters to inches)
        Pose2d currentPose = new Pose2d(
            botPose.getPosition().x * 39.37,  // Convert X from meters to inches
            botPose.getPosition().y * 39.37,  // Convert Y from meters to inches
            Math.toRadians(botPose.getOrientation().getYaw())
        );
        
        // Set RoadRunner's pose estimate
        drive.setPoseEstimate(currentPose);
        telemetry.addData("Current Pose", String.format("X: %.1f, Y: %.1f, Heading: %.1f°", 
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
        
        // Add more detailed pose information (converted to inches)
        telemetry.addData("Limelight Pose Details", "");
        telemetry.addData("BotPose X", String.format("%.1f inches", botPose.getPosition().x * 39.37));
        telemetry.addData("BotPose Y", String.format("%.1f inches", botPose.getPosition().y * 39.37));
        telemetry.addData("BotPose Z", String.format("%.1f inches", botPose.getPosition().z * 39.37));
        telemetry.addData("BotPose Yaw", String.format("%.1f degrees", botPose.getOrientation().getYaw()));
        
        // Show raw camera data for comparison
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("--- Raw Camera Data ---", "");
            telemetry.addData("TX (degrees)", String.format("%.2f", llResult.getTx()));
            telemetry.addData("TY (degrees)", String.format("%.2f", llResult.getTy()));
            telemetry.addData("TA (area)", String.format("%.2f", llResult.getTa()));
            
            // Calculate rough distance from raw data (approximate)
            double rawAngularDistance = Math.sqrt(
                llResult.getTx() * llResult.getTx() + llResult.getTy() * llResult.getTy()
            );
            telemetry.addData("Raw Angular Distance", String.format("%.2f degrees", rawAngularDistance));
            
            // Very rough distance estimate (not accurate, just for comparison)
            double roughDistanceEstimate = (rawAngularDistance * 12.0) / 5.0; // Very rough approximation
            telemetry.addData("Rough Distance Est", String.format("%.1f inches", roughDistanceEstimate));
            
            // More detailed distance analysis
            telemetry.addData("--- Distance Analysis ---", "");
            telemetry.addData("Actual Distance", "110 inches (9.17 feet)");
            telemetry.addData("Calculated Distance", String.format("%.1f inches (%.1f feet)", 
                Math.sqrt((botPose.getPosition().x * 39.37) * (botPose.getPosition().x * 39.37) + 
                         (botPose.getPosition().y * 39.37) * (botPose.getPosition().y * 39.37)),
                Math.sqrt((botPose.getPosition().x * 39.37) * (botPose.getPosition().x * 39.37) + 
                         (botPose.getPosition().y * 39.37) * (botPose.getPosition().y * 39.37)) / 12.0));
            
            // Calculate the ratio to identify the issue
            double actualDistance = 110.0; // inches
            double calculatedDistance = Math.sqrt((botPose.getPosition().x * 39.37) * (botPose.getPosition().x * 39.37) + 
                                                 (botPose.getPosition().y * 39.37) * (botPose.getPosition().y * 39.37));
            double ratio = calculatedDistance / actualDistance;
            telemetry.addData("Distance Ratio", String.format("%.2f (should be 1.0)", ratio));
            
            if (ratio > 1.5) {
                telemetry.addData("Issue", "Calculated distance too large - check AprilTag size config");
            } else if (ratio < 0.7) {
                telemetry.addData("Issue", "Calculated distance too small - check AprilTag size config");
            } else {
                telemetry.addData("Issue", "Distance looks reasonable");
            }
            
            // DIAGNOSTIC: Check if this looks like tag-relative vs field-relative
            telemetry.addData("--- DIAGNOSTIC ---", "");
            // Convert to inches for comparison (since we know limelight gives meters)
            double xInches = botPose.getPosition().x * 39.37;
            double yInches = botPose.getPosition().y * 39.37;
            if (Math.abs(xInches) < 10 && Math.abs(yInches) < 10) {
                telemetry.addData("WARNING", "Pose values are very small!");
                telemetry.addData("Likely Issue", "Tag-relative mode or wrong units");
            } else {
                telemetry.addData("Pose values", "Look reasonable for field coordinates");
            }
            telemetry.addData("Raw Meters", String.format("X: %.2f, Y: %.2f", botPose.getPosition().x, botPose.getPosition().y));
            telemetry.addData("Converted Inches", String.format("X: %.1f, Y: %.1f", xInches, yInches));
            
            // Check if we're getting reasonable angular data
            if (Math.abs(llResult.getTx()) < 5 && Math.abs(llResult.getTy()) < 5) {
                telemetry.addData("Angular Data", "Very small angles - robot very close to tag");
            } else {
                telemetry.addData("Angular Data", "Reasonable angles for 10+ feet distance");
            }
        }
        
        // Calculate and display distance from origin (tag position) - convert meters to inches
        double distanceFromOrigin = Math.sqrt(
            (botPose.getPosition().x * 39.37) * (botPose.getPosition().x * 39.37) + 
            (botPose.getPosition().y * 39.37) * (botPose.getPosition().y * 39.37)
        );
        telemetry.addData("Distance from Origin", String.format("%.1f inches (%.1f feet)", 
            distanceFromOrigin, distanceFromOrigin / 12.0));
        
        // Check if this matches expected distance
        if (Math.abs(distanceFromOrigin - 120.0) < 5.0) {
            telemetry.addData("Distance Check", "CORRECT - matches 10 feet");
        } else {
            telemetry.addData("Distance Check", "INCORRECT - expected ~120 inches");
        }

        // Use relative navigation - move closer to the tag by a fixed distance
        double targetDistance = 60.0; // 5 feet in inches
        double moveDistance = distanceFromOrigin - targetDistance; // How much closer to move
        
        // Calculate the direction from robot to tag (unit vector)
        double directionX = -(botPose.getPosition().x * 39.37) / distanceFromOrigin; // Negative because tag is at origin
        double directionY = -(botPose.getPosition().y * 39.37) / distanceFromOrigin;
        
        // Calculate target position by moving closer to the tag
        double targetX = currentPose.getX() + (directionX * moveDistance);
        double targetY = currentPose.getY() + (directionY * moveDistance);
        
        // Create target pose - face the tag (which is at origin)
        Pose2d targetPose = new Pose2d(targetX, targetY, 
            Math.atan2(-targetY, -targetX)); // Face the tag (which is at origin)
        
        telemetry.addData("--- Relative Navigation ---", "");
        telemetry.addData("Current Distance", String.format("%.1f inches", distanceFromOrigin));
        telemetry.addData("Target Distance", String.format("%.1f inches", targetDistance));
        telemetry.addData("Move Distance", String.format("%.1f inches", moveDistance));
        telemetry.addData("Direction Vector", String.format("(%.3f, %.3f)", directionX, directionY));
        telemetry.addData("Target X", String.format("%.1f inches", targetX));
        telemetry.addData("Target Y", String.format("%.1f inches", targetY));
        telemetry.addData("Target Pose", String.format("X: %.1f, Y: %.1f, Heading: %.1f°", 
            targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading())));
        
        // Show the actual movement calculation
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();
        telemetry.addData("Movement", String.format("ΔX: %.1f, ΔY: %.1f", deltaX, deltaY));
        telemetry.addData("Movement Distance", String.format("%.1f inches", Math.sqrt(deltaX*deltaX + deltaY*deltaY)));
        
        // Calculate distance to target
        double distanceToTarget = Math.sqrt(
            Math.pow(targetPose.getX() - currentPose.getX(), 2) + 
            Math.pow(targetPose.getY() - currentPose.getY(), 2)
        );
        telemetry.addData("Distance to Target", String.format("%.1f inches", distanceToTarget));
        
        // Safety check: if the distance seems unreasonable, don't navigate
        if (distanceToTarget > 200.0) { // More than ~16 feet seems unreasonable
            telemetry.addData("*** DISTANCE TOO LARGE ***", "");
            telemetry.addData("Distance", String.format("%.1f inches (%.1f feet)", distanceToTarget, distanceToTarget/12.0));
            telemetry.addData("Action", "Skipping navigation - distance seems incorrect");
            navigationInProgress = false;
            return;
        }
        
        // Check if we're close enough to target (within 3 inches)
        if (distanceToTarget < 3.0) {
            telemetry.addData("*** TARGET REACHED ***", "");
            telemetry.addData("Navigation", "COMPLETED");
            navigationInProgress = false;
            navigationCompleted = true;
            return;
        }
        
        // Build and execute trajectory
        telemetry.addData("*** EXECUTING TRAJECTORY ***", "");
        telemetry.addData("From", String.format("(%.1f, %.1f)", currentPose.getX(), currentPose.getY()));
        telemetry.addData("To", String.format("(%.1f, %.1f)", targetPose.getX(), targetPose.getY()));
        telemetry.addData("Current Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
        telemetry.addData("Target Heading", String.format("%.1f°", Math.toDegrees(targetPose.getHeading())));
        
        // Check if the target is very close (might cause rotation only)
        double distanceToTargetCheck = Math.sqrt(
            Math.pow(targetPose.getX() - currentPose.getX(), 2) + 
            Math.pow(targetPose.getY() - currentPose.getY(), 2)
        );
        telemetry.addData("Distance to Target", String.format("%.1f inches", distanceToTargetCheck));
        
        if (distanceToTargetCheck < 5.0) {
            telemetry.addData("*** TARGET TOO CLOSE ***", "");
            telemetry.addData("Action", "Skipping navigation - target too close");
            navigationInProgress = false;
            return;
        }
        
        try {
            // Try a simpler approach - just move to the target position without changing heading
            telemetry.addData("Using", "lineTo (no heading change)");
            drive.followTrajectory(
                drive.trajectoryBuilder(currentPose)
                    .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(targetPose.getX(), targetPose.getY()))
                    .build()
            );
            
            telemetry.addData("*** TRAJECTORY COMPLETE ***", "");
            navigationInProgress = false;
            navigationCompleted = true;
            
        } catch (Exception e) {
            telemetry.addData("*** TRAJECTORY ERROR ***", "");
            telemetry.addData("Error", e.getMessage());
            telemetry.addData("Action", "Stopping navigation");
            navigationInProgress = false;
        }
    }
    
    /**
     * Calculate the target pose 5 feet in front of the AprilTag.
     * The limelight pose gives us the robot's position relative to the AprilTag.
     * We want to position the robot 5 feet away from the tag, maintaining the same direction.
     * 
     * @param robotPose The robot's current pose from limelight (relative to AprilTag)
     * @return The target pose 5 feet in front of the AprilTag
     */
    private Pose2d calculateTargetPoseInFrontOfTag(Pose3D robotPose) {
        // Current robot position relative to AprilTag (convert meters to inches)
        double robotX = robotPose.getPosition().x * 39.37;
        double robotY = robotPose.getPosition().y * 39.37;
        
        // Calculate the direction from the tag to the robot
        // Since robotPose is relative to tag, tag is at (0,0) relative to robot
        double dx = robotX - 0.0;  // robotX is already relative to tag
        double dy = robotY - 0.0;  // robotY is already relative to tag
        
        // Distance from tag to robot
        double distanceToRobot = Math.sqrt(dx * dx + dy * dy);
        
        telemetry.addData("Debug Info", "");
        telemetry.addData("Robot X (relative to tag)", String.format("%.1f inches", robotX));
        telemetry.addData("Robot Y (relative to tag)", String.format("%.1f inches", robotY));
        telemetry.addData("Distance to tag", String.format("%.1f inches", distanceToRobot));
        
        // Additional debugging for fieldmap issues
        telemetry.addData("Fieldmap Debug", "");
        if (Math.abs(robotX) < 10 && Math.abs(robotY) < 10) {
            telemetry.addData("Warning", "Robot appears very close to tag!");
            telemetry.addData("Possible Issue", "Fieldmap may be incorrect");
        }
        if (distanceToRobot < 20) {
            telemetry.addData("Warning", "Distance seems too small for 10 feet");
            telemetry.addData("Check", "Fieldmap tag positions vs actual positions");
        }
        
        if (distanceToRobot == 0) {
            // If robot is exactly at tag position, default to a position 5 feet away
            return new Pose2d(TARGET_DISTANCE_INCHES, 0, 0);
        }
        
        // Unit vector from tag to robot
        double unitX = dx / distanceToRobot;
        double unitY = dy / distanceToRobot;
        
        // Calculate target position: 5 feet away from tag in the same direction
        // Since we're working in tag-relative coordinates, target is just the unit vector * distance
        double targetX = unitX * TARGET_DISTANCE_INCHES;
        double targetY = unitY * TARGET_DISTANCE_INCHES;
        
        // Calculate target heading: robot should face the tag
        // Heading is the angle from the robot to the tag (which is at 0,0)
        double targetHeading = Math.atan2(0 - targetY, 0 - targetX);
        
        telemetry.addData("Target X", String.format("%.1f inches", targetX));
        telemetry.addData("Target Y", String.format("%.1f inches", targetY));
        telemetry.addData("Target Heading", String.format("%.1f degrees", Math.toDegrees(targetHeading)));
        
        return new Pose2d(targetX, targetY, targetHeading);
    }
    
    private void handlePoseLoss(long currentTime) {
        long timeSinceLastPose = currentTime - lastValidPoseTime;
        
        telemetry.addData("--- Pose Loss Handling ---", "");
        telemetry.addData("Time since last pose", String.format("%d ms", timeSinceLastPose));
        telemetry.addData("Tag recently detected", tagRecentlyDetected);
        
        if (tagRecentlyDetected && timeSinceLastPose < POSE_TIMEOUT_MS && lastValidPose != null) {
            // Continue with last known pose for a short time
            telemetry.addData("Using last known pose", "Continuing navigation");
            telemetry.addData("Last pose age", String.format("%d ms", timeSinceLastPose));
            executeRoadRunnerNavigation(lastValidPose);
        } else {
            // Stop navigation if pose has been lost for too long
            telemetry.addData("Pose lost too long", "Stopping navigation");
            telemetry.addData("Action", "Waiting for AprilTag to come back into view");
            navigationInProgress = false;
            tagRecentlyDetected = false;
        }
    }
}
