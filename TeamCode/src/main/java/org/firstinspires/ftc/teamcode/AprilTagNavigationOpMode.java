package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class AprilTagNavigationOpMode extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    
    // Constants for AprilTag navigation
    private static final int TARGET_APRILTAG_ID = 20;
    private static final double TARGET_DISTANCE_FEET = 3.0; // 3 feet in front of tag
    private static final double TARGET_DISTANCE_INCHES = TARGET_DISTANCE_FEET * 12.0;
    private static final double ROTATION_SPEED = 0.4; // Speed for turning when tag not in view
    private static final double FINE_ROTATION_SPEED = 0.15; // Slower speed for fine adjustments when tag is in view
    private static final double MOVEMENT_SPEED = 0.4; // Speed for moving towards tag
    private static final double POSITION_TOLERANCE = 2.0; // Tolerance in inches
    private static final double ANGLE_TOLERANCE = 5.0; // Tolerance in degrees
    private static final double FINE_ANGLE_TOLERANCE = 1.0; // Very tight tolerance for fine adjustments
    
    // State variables
    private boolean navigationActive = false;
    private boolean lastButtonState = false;
    private int loopCounter = 0;
    private long lastTagDetectionTime = 0;
    private boolean tagRecentlyDetected = false;
    private int detectionHistoryCount = 0;
    private boolean isNavigatingToTag = false; // New state to track if we're actively navigating

    @Override
    public void runOpMode() throws InterruptedException {
        // Use default telemetry settings for stability
        
        telemetry.addData("AprilTag Navigation OpMode", "Initializing...");
        telemetry.update();

        // Initialize hardware - same as LimelightOpMode1
        motorFrontLeft = hardwareMap.dcMotor.get("frontleft");
        motorBackLeft = hardwareMap.dcMotor.get("backleft");
        motorFrontRight = hardwareMap.dcMotor.get("frontright");
        motorBackRight = hardwareMap.dcMotor.get("backright");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set limelight to pipeline 0 (configured 0");
        telemetry.update();

        telemetry.addData("Hardware initialized", "Ready");
        telemetry.addData("Press A button to start navigation to AprilTag ID", TARGET_APRILTAG_ID);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Start limelight
        limelight.start();
        telemetry.addData("Limelight started", "Pipeline 9 active");
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
            loopCounter++;
            long currentTime = System.currentTimeMillis();
            
            // Clear telemetry for fresh display
            telemetry.clear();
            
            // Add header information
            telemetry.addData("=== AprilTag Navigation Debug ===", "");
            telemetry.addData("Loop Counter", loopCounter);
            telemetry.addData("Navigation Active", navigationActive);
            telemetry.addData("A Button Pressed", gamepad1.a);
            telemetry.addData("Is Navigating to Tag", isNavigatingToTag);
            telemetry.addData("Tag Recently Detected", tagRecentlyDetected);
            
            // Check for button press to start/stop navigation
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                // Button just pressed
                navigationActive = !navigationActive;
                if (navigationActive) {
                    telemetry.addData("*** Navigation STARTED ***", "Looking for AprilTag ID " + TARGET_APRILTAG_ID);
                } else {
                    telemetry.addData("*** Navigation STOPPED ***", "Returning to manual control");
                    stopAllMotors();
                }
            }
            lastButtonState = currentButtonState;

            if (navigationActive) {
                performAprilTagNavigation();
            } else {
                // Manual control when navigation is not active
                performManualControl();
                // Always show AprilTag detection debug info
                performAprilTagDebug();
            }
            
            // Track detection history for debugging
            trackDetectionHistory();

            // Update telemetry
            telemetry.update();
            
            // No sleep - maximum responsiveness for AprilTag detection
        }
    }

    private void performAprilTagNavigation() {
        telemetry.addData("--- Navigation Status ---", "");
        
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Limelight Result", result != null ? "Received" : "NULL");
        
        if (result != null) {
            telemetry.addData("Result Valid", result.isValid());
            telemetry.addData("Result TX", String.format("%.2f", result.getTx()));
            telemetry.addData("Result TY", String.format("%.2f", result.getTy()));
            telemetry.addData("Result TA", String.format("%.2f", result.getTa()));
            
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                telemetry.addData("Fiducials Detected", fiducialResults.size());
                
                // Display all detected fiducials
                for (int i = 0; i < fiducialResults.size(); i++) {
                    LLResultTypes.FiducialResult fiducial = fiducialResults.get(i);
                    telemetry.addData("Fiducial " + i, String.format("ID: %d, X: %.2f, Y: %.2f", 
                        fiducial.getFiducialId(), fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees()));
                }
                
                // Look for our target AprilTag ID
                LLResultTypes.FiducialResult targetTag = null;
                for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                    if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                        targetTag = fiducial;
                        break;
                    }
                }

                if (targetTag != null) {
                    telemetry.addData("*** TARGET TAG FOUND ***", "ID: " + TARGET_APRILTAG_ID);
                    telemetry.addData("Tag X Position", String.format("%.2f°", targetTag.getTargetXDegrees()));
                    telemetry.addData("Tag Y Position", String.format("%.2f°", targetTag.getTargetYDegrees()));
                    lastTagDetectionTime = System.currentTimeMillis();
                    tagRecentlyDetected = true;
                    isNavigatingToTag = true; // Set navigation state
                    telemetry.addData("DEBUG: Calling navigateToTag()", "Tag detected - should NOT rotate");
                    // Tag found! Navigate to it
                    navigateToTag(targetTag);
                } else {
                    // Check if we recently detected the tag (within last 1000ms - longer grace period)
                    long currentTime = System.currentTimeMillis();
                    if (tagRecentlyDetected && (currentTime - lastTagDetectionTime) < 1000) {
                        telemetry.addData("Target Tag Status", "Recently detected, continuing navigation");
                        telemetry.addData("Time since detection", String.format("%d ms", currentTime - lastTagDetectionTime));
                        telemetry.addData("Navigation State", "Holding position - tag temporarily out of view");
                        telemetry.addData("DEBUG: STOPPING ALL MOTORS", "Tag recently detected - no rotation");
                        // Continue with last known navigation or stop motors
                        stopAllMotors();
                    } else {
                        tagRecentlyDetected = false;
                        isNavigatingToTag = false; // Clear navigation state
                        telemetry.addData("Target Tag Status", "AprilTag ID " + TARGET_APRILTAG_ID + " not in view");
                        telemetry.addData("Available Tag IDs", getAvailableTagIds(fiducialResults));
                        telemetry.addData("Action", "Rotating to find tag");
                        telemetry.addData("DEBUG: Calling rotateToFindTag()", "No tag detected - will rotate");
                        rotateToFindTag();
                    }
                }
            } else {
                telemetry.addData("Limelight Status", "Invalid result - rotating");
                telemetry.addData("DEBUG: Calling rotateToFindTag()", "Invalid result - will rotate");
                rotateToFindTag();
            }
        } else {
            telemetry.addData("Limelight Status", "No result - rotating");
            telemetry.addData("DEBUG: Calling rotateToFindTag()", "No result - will rotate");
            rotateToFindTag();
        }
    }

    private void navigateToTag(LLResultTypes.FiducialResult tag) {
        telemetry.addData("--- Navigating to Tag ---", "");
        telemetry.addData("Navigation State", "ACTIVE - Tag detected");
        telemetry.addData("DEBUG: NO ROTATION MODE", "This method should NEVER rotate!");
        
        // Get tag position data from the specific fiducial result
        double tx = tag.getTargetXDegrees(); // Horizontal offset from center (-29.8 to 29.8 degrees)
        double ty = tag.getTargetYDegrees(); // Vertical offset from center (-24.85 to 24.85 degrees)
        
        // Get the main result for area calculation
        LLResult result = limelight.getLatestResult();
        double ta = result.getTa(); // Target area (0-100% of image)
        
        telemetry.addData("Target Tag ID", tag.getFiducialId());
        telemetry.addData("TX (horizontal)", String.format("%.2f°", tx));
        telemetry.addData("TY (vertical)", String.format("%.2f°", ty));
        telemetry.addData("TA (area)", String.format("%.2f%%", ta));

        // Calculate distance from tag (rough estimation based on area)
        double estimatedDistance = calculateDistanceFromArea(ta);
        
        telemetry.addData("Estimated Distance", String.format("%.1f inches", estimatedDistance));
        telemetry.addData("Target Distance", String.format("%.1f inches", TARGET_DISTANCE_INCHES));

        // Calculate movement needed
        double distanceError = estimatedDistance - TARGET_DISTANCE_INCHES;
        double angleError = tx; // Horizontal offset is our angle error

        telemetry.addData("Distance Error", String.format("%.1f inches", distanceError));
        telemetry.addData("Angle Error", String.format("%.1f°", angleError));
        telemetry.addData("Position Tolerance", String.format("%.1f inches", POSITION_TOLERANCE));
        telemetry.addData("Angle Tolerance", String.format("%.1f°", ANGLE_TOLERANCE));

        // Check if we're close enough to target position
        if (Math.abs(distanceError) < POSITION_TOLERANCE && Math.abs(angleError) < ANGLE_TOLERANCE) {
            telemetry.addData("*** ARRIVED ***", "At target position!");
            telemetry.addData("Distance OK", Math.abs(distanceError) < POSITION_TOLERANCE);
            telemetry.addData("Angle OK", Math.abs(angleError) < ANGLE_TOLERANCE);
            stopAllMotors();
            return;
        }

        // SIMPLIFIED APPROACH: NO ROTATION AT ALL - ONLY FORWARD/BACKWARD MOVEMENT
        double forwardPower = 0;
        double strafePower = 0;
        double rotatePower = 0; // ALWAYS ZERO - NO ROTATION

        // Only move forward/backward based on distance
        if (Math.abs(distanceError) > POSITION_TOLERANCE) {
            forwardPower = Math.max(-MOVEMENT_SPEED, Math.min(MOVEMENT_SPEED, 
                distanceError / TARGET_DISTANCE_INCHES * MOVEMENT_SPEED));
            telemetry.addData("Forward Movement", String.format("Moving %.3f (error: %.1f inches)", forwardPower, distanceError));
        } else {
            telemetry.addData("Forward Movement", "Distance OK - no movement needed");
        }

        telemetry.addData("Final Motor Powers", "");
        telemetry.addData("Forward Power", String.format("%.3f", forwardPower));
        telemetry.addData("Strafe Power", String.format("%.3f", strafePower));
        telemetry.addData("Rotate Power", String.format("%.3f", rotatePower));
        telemetry.addData("ROTATION POWER", "ALWAYS 0 - NO ROTATION!");

        // Apply movement - NO ROTATION
        setMotorPowers(forwardPower, strafePower, rotatePower);
        
        // Action description
        if (Math.abs(distanceError) > POSITION_TOLERANCE) {
            telemetry.addData("Action", distanceError > 0 ? "Moving forward (too far)" : "Moving backward (too close)");
        } else {
            telemetry.addData("Action", "Positioning complete - holding position");
        }
        
        // Debug information
        telemetry.addData("Debug Info", "");
        telemetry.addData("Angle Error", String.format("%.2f°", angleError));
        telemetry.addData("Rotation Applied", "NEVER - Always 0");
        telemetry.addData("Will Move Forward", Math.abs(distanceError) > POSITION_TOLERANCE ? "YES" : "NO");
    }

    private void rotateToFindTag() {
        telemetry.addData("--- Searching for Tag ---", "");
        telemetry.addData("Action", "Rotating slowly to find AprilTag ID " + TARGET_APRILTAG_ID);
        telemetry.addData("Rotation Speed", String.format("%.2f", ROTATION_SPEED));
        
        // Check for target tag detection before rotating
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            telemetry.addData("While Rotating - Fiducials", fiducialResults.size());
            if (fiducialResults.size() > 0) {
                telemetry.addData("While Rotating - Available IDs", getAvailableTagIds(fiducialResults));
            }
            
            // Check if target is detected
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                    telemetry.addData("*** TARGET DETECTED WHILE ROTATING ***", "ID: " + TARGET_APRILTAG_ID);
                    telemetry.addData("Rotating - Tag X", String.format("%.2f°", fiducial.getTargetXDegrees()));
                    telemetry.addData("Rotating - Tag Y", String.format("%.2f°", fiducial.getTargetYDegrees()));
                    // Stop rotation immediately when target is found
                    stopAllMotors();
                    return; // Exit immediately - don't rotate
                }
            }
        }
        
        // Only rotate if target not found
        telemetry.addData("Target Not Found", "Continuing rotation");
        setMotorPowers(0, 0, ROTATION_SPEED);
    }

    private double calculateDistanceFromArea(double area) {
        // This is a simplified distance calculation based on tag area
        // You may need to calibrate these values for your specific setup
        // Generally, larger area = closer distance
        if (area > 0) {
            // Calibrated estimation: area of 15% corresponds to about 3 feet (36 inches)
            // Formula: distance = sqrt(calibration_area / current_area) * calibration_distance
            double calibrationArea = 15.0; // 15% area at calibration distance
            double calibrationDistance = 36.0; // 3 feet in inches
            return Math.sqrt(calibrationArea / area) * calibrationDistance;
        }
        return 100.0; // Default large distance if no area data
    }

    private void setMotorPowers(double forward, double strafe, double rotate) {
        // Mecanum wheel calculations
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize powers to prevent exceeding 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                                  Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        
        boolean normalized = false;
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            normalized = true;
        }

        // Add motor power telemetry
        telemetry.addData("--- Motor Powers ---", "");
        telemetry.addData("Front Left", String.format("%.3f", frontLeftPower));
        telemetry.addData("Back Left", String.format("%.3f", backLeftPower));
        telemetry.addData("Front Right", String.format("%.3f", frontRightPower));
        telemetry.addData("Back Right", String.format("%.3f", backRightPower));
        telemetry.addData("Max Power", String.format("%.3f", maxPower));
        telemetry.addData("Normalized", normalized);

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    private void stopAllMotors() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void performAprilTagDebug() {
        telemetry.addData("--- AprilTag Debug (Stationary) ---", "");
        
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Limelight Result", result != null ? "Received" : "NULL");
        
        if (result != null) {
            telemetry.addData("Result Valid", result.isValid());
            telemetry.addData("Result TX", String.format("%.2f", result.getTx()));
            telemetry.addData("Result TY", String.format("%.2f", result.getTy()));
            telemetry.addData("Result TA", String.format("%.2f", result.getTa()));
            // Simplified debug - avoid telemetry overflow
            telemetry.addData("Result Type", result.getClass().getSimpleName());
            
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                telemetry.addData("Fiducials Detected", fiducialResults.size());
                
                if (fiducialResults.size() > 0) {
                    telemetry.addData("Available Tag IDs", getAvailableTagIds(fiducialResults));
                    
                    // Display all detected fiducials with detailed info
                    for (int i = 0; i < fiducialResults.size(); i++) {
                        LLResultTypes.FiducialResult fiducial = fiducialResults.get(i);
                        telemetry.addData("Tag " + i + " ID", fiducial.getFiducialId());
                        telemetry.addData("Tag " + i + " X", String.format("%.2f°", fiducial.getTargetXDegrees()));
                        telemetry.addData("Tag " + i + " Y", String.format("%.2f°", fiducial.getTargetYDegrees()));
                        
                        // Highlight if this is our target tag
                        if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                            telemetry.addData("*** TARGET FOUND ***", "ID: " + TARGET_APRILTAG_ID);
                            telemetry.addData("Target X Position", String.format("%.2f°", fiducial.getTargetXDegrees()));
                            telemetry.addData("Target Y Position", String.format("%.2f°", fiducial.getTargetYDegrees()));
                        }
                    }
                } else {
                    telemetry.addData("No AprilTags Detected", "Check camera view and lighting");
                }
            } else {
                telemetry.addData("Limelight Status", "Invalid result - check camera connection");
            }
        } else {
            telemetry.addData("Limelight Status", "No result - check camera connection");
        }
        
        telemetry.addData("Target Tag ID", TARGET_APRILTAG_ID);
        telemetry.addData("Pipeline", "9 (AprilTag IDs 20-24)");
    }

    private void trackDetectionHistory() {
        long currentTime = System.currentTimeMillis();
        
        // Check for target tag detection every loop
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                    detectionHistoryCount++;
                    lastTagDetectionTime = currentTime;
                    tagRecentlyDetected = true;
                    break;
                }
            }
        }
        
        // Show detection history in telemetry
        telemetry.addData("Detection History", "Target detected " + detectionHistoryCount + " times");
        if (tagRecentlyDetected) {
            long timeSinceDetection = currentTime - lastTagDetectionTime;
            telemetry.addData("Last Detection", String.format("%d ms ago", timeSinceDetection));
        }
    }

    private String getAvailableTagIds(List<LLResultTypes.FiducialResult> fiducialResults) {
        if (fiducialResults.isEmpty()) {
            return "None";
        }
        
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < fiducialResults.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(fiducialResults.get(i).getFiducialId());
        }
        return sb.toString();
    }

    private void performManualControl() {
        telemetry.addData("--- Manual Control ---", "");
        
        // Manual control when navigation is not active
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        telemetry.addData("Gamepad Input", "");
        telemetry.addData("Left Stick Y", String.format("%.2f", gamepad1.left_stick_y));
        telemetry.addData("Left Stick X", String.format("%.2f", gamepad1.left_stick_x));
        telemetry.addData("Right Stick X", String.format("%.2f", gamepad1.right_stick_x));
        telemetry.addData("Processed Y", String.format("%.2f", y));
        telemetry.addData("Processed X", String.format("%.2f", x));
        telemetry.addData("Processed RX", String.format("%.2f", rx));

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        telemetry.addData("Manual Motor Powers", "");
        telemetry.addData("Front Left", String.format("%.3f", frontLeftPower));
        telemetry.addData("Back Left", String.format("%.3f", backLeftPower));
        telemetry.addData("Front Right", String.format("%.3f", frontRightPower));
        telemetry.addData("Back Right", String.format("%.3f", backRightPower));

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Instructions", "Use left stick to drive, right stick X to rotate");
        telemetry.addData("Press A", "to start AprilTag navigation");
        telemetry.addData("Debug Info", "AprilTag detection shown below when stationary");
    }
}
