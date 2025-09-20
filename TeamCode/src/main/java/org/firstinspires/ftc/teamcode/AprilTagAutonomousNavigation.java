package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AprilTag Auto Navigation", group = "Autonomous")
public class AprilTagAutonomousNavigation extends LinearOpMode {

    // Hardware declarations
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    
    // Navigation constants
    private static final int TARGET_APRILTAG_ID = 20;
    private static final double TARGET_DISTANCE_FEET = 5.0;
    private static final double TARGET_DISTANCE_INCHES = TARGET_DISTANCE_FEET * 12.0;
    private static final double POSITION_TOLERANCE = 2.0; // inches
    private static final double ANGLE_TOLERANCE = 3.0; // degrees
    private static final double MOVEMENT_SPEED = 0.2; // Reduced for smoother movement
    private static final double ROTATION_SPEED = 0.15; // Reduced for smoother rotation
    
    // State variables
    private boolean targetReached = false;
    private int loopCounter = 0;
    private long lastValidPoseTime = 0;
    private boolean tagRecentlyDetected = false;
    private Pose3D lastValidPose = null;
    private static final long POSE_TIMEOUT_MS = 2000; // 2 seconds grace period

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("AprilTag Autonomous Navigation", "Initializing...");
        telemetry.update();
        
        // Initialize hardware
        initializeHardware();
        
        telemetry.addData("Hardware initialized", "Ready to start");
        telemetry.addData("Target distance", TARGET_DISTANCE_FEET + " feet");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Start limelight
        limelight.start();
        telemetry.addData("Limelight started", "Beginning navigation");
        telemetry.update();
        
        // Give limelight time to initialize
        sleep(1000);
        
        // Main navigation loop
        while (opModeIsActive() && !targetReached) {
            loopCounter++;
            
            // Clear telemetry for fresh display
            telemetry.clear();
            telemetry.addData("=== AprilTag Autonomous Navigation ===", "");
            telemetry.addData("Loop Counter", loopCounter);
            telemetry.addData("Target Reached", targetReached);
            
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
                        telemetry.addData("Robot X", String.format("%.1f inches", botPose.getPosition().x));
                        telemetry.addData("Robot Y", String.format("%.1f inches", botPose.getPosition().y));
                        telemetry.addData("Robot Z", String.format("%.1f inches", botPose.getPosition().z));
                        telemetry.addData("Robot Yaw", String.format("%.1f degrees", botPose.getOrientation().getYaw()));
                        
                        // Update pose tracking
                        lastValidPose = botPose;
                        lastValidPoseTime = currentTime;
                        tagRecentlyDetected = true;
                        
                        // Perform navigation
                        performNavigation(botPose);
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
            
            telemetry.update();
        }
        
        // Final stop
        stopAllMotors();
        telemetry.addData("Navigation Complete", "Target reached or opmode stopped");
        telemetry.update();
    }
    
    private void initializeHardware() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use pipeline 0 for AprilTag detection
        
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        
        // Initialize motors
        motorFrontLeft = hardwareMap.dcMotor.get("frontleft");
        motorBackLeft = hardwareMap.dcMotor.get("backleft");
        motorFrontRight = hardwareMap.dcMotor.get("frontright");
        motorBackRight = hardwareMap.dcMotor.get("backright");
        
        // Reverse right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    private void updateRobotOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        
        telemetry.addData("IMU Data", "");
        telemetry.addData("Yaw", String.format("%.1f degrees", orientation.getYaw()));
        telemetry.addData("Pitch", String.format("%.1f degrees", orientation.getPitch()));
        telemetry.addData("Roll", String.format("%.1f degrees", orientation.getRoll()));
    }
    
    private void performNavigation(Pose3D botPose) {
        telemetry.addData("--- Navigation Analysis ---", "");
        
        // Calculate distance from origin (assuming tag is at origin)
        double currentDistance = Math.sqrt(botPose.getPosition().x * botPose.getPosition().x + botPose.getPosition().y * botPose.getPosition().y);
        double distanceError = currentDistance - TARGET_DISTANCE_INCHES;
        
        // Calculate angle error (robot should face the tag)
        double currentYaw = botPose.getOrientation().getYaw();
        // Calculate the direction from robot to tag (tag is at origin)
        double targetYaw = Math.toDegrees(Math.atan2(-botPose.getPosition().y, -botPose.getPosition().x));
        double angleError = normalizeAngle(targetYaw - currentYaw);
        
        // Be very conservative about rotation to avoid losing tag
        if (Math.abs(angleError) < 20.0 && Math.abs(distanceError) > POSITION_TOLERANCE) {
            angleError = 0; // Don't rotate if we're roughly facing the tag and need to move forward/back
        }
        
        telemetry.addData("Current Distance", String.format("%.1f inches", currentDistance));
        telemetry.addData("Target Distance", String.format("%.1f inches", TARGET_DISTANCE_INCHES));
        telemetry.addData("Distance Error", String.format("%.1f inches", distanceError));
        telemetry.addData("Current Yaw", String.format("%.1f degrees", currentYaw));
        telemetry.addData("Target Yaw", String.format("%.1f degrees", targetYaw));
        telemetry.addData("Angle Error", String.format("%.1f degrees", angleError));
        
        // Priority: Distance correction first, then angle correction
        boolean needsDistanceCorrection = Math.abs(distanceError) > POSITION_TOLERANCE;
        boolean needsAngleCorrection = Math.abs(angleError) > ANGLE_TOLERANCE;
        
        telemetry.addData("Needs Distance Correction", needsDistanceCorrection);
        telemetry.addData("Needs Angle Correction", needsAngleCorrection);
        
        // Check if target position is reached
        if (Math.abs(distanceError) < POSITION_TOLERANCE && Math.abs(angleError) < ANGLE_TOLERANCE) {
            telemetry.addData("*** TARGET REACHED ***", "");
            telemetry.addData("Distance OK", Math.abs(distanceError) < POSITION_TOLERANCE);
            telemetry.addData("Angle OK", Math.abs(angleError) < ANGLE_TOLERANCE);
            stopAllMotors();
            targetReached = true;
            return;
        }
        
        // Calculate movement needed
        double forwardPower = 0;
        double strafePower = 0;
        double rotatePower = 0;
        
        // PRIORITY: Distance correction first, avoid rotation unless absolutely necessary
        if (needsDistanceCorrection && Math.abs(angleError) < 25.0) {
            // Move straight toward/away from tag - prioritize distance over angle
            double moveDirection = -Math.signum(distanceError); // Negative because we want to move toward origin
            double speedScale = Math.min(1.0, Math.abs(distanceError) / 24.0);
            forwardPower = moveDirection * MOVEMENT_SPEED * speedScale;
            telemetry.addData("Priority", "Distance correction (straight movement)");
        }
        // Only rotate if angle error is very large AND we're close to target distance
        else if (needsAngleCorrection && Math.abs(angleError) > 30.0 && Math.abs(distanceError) < 12.0) {
            double rotationScale = Math.min(1.0, Math.abs(angleError) / 60.0);
            rotatePower = Math.signum(angleError) * ROTATION_SPEED * rotationScale;
            telemetry.addData("Priority", "Angle correction (large angle, close distance)");
        }
        // If we need distance correction and angle is acceptable, do distance correction
        else if (needsDistanceCorrection) {
            double moveDirection = -Math.signum(distanceError);
            double speedScale = Math.min(1.0, Math.abs(distanceError) / 24.0);
            forwardPower = moveDirection * MOVEMENT_SPEED * speedScale;
            telemetry.addData("Priority", "Distance correction (avoiding rotation)");
        }
        // Only do angle correction if distance is very close to target
        else if (needsAngleCorrection && Math.abs(distanceError) < 6.0) {
            double rotationScale = Math.min(1.0, Math.abs(angleError) / 60.0);
            rotatePower = Math.signum(angleError) * ROTATION_SPEED * rotationScale;
            telemetry.addData("Priority", "Fine angle correction (very close to target)");
        }
        
        telemetry.addData("--- Motor Commands ---", "");
        telemetry.addData("Forward Power", String.format("%.3f", forwardPower));
        telemetry.addData("Strafe Power", String.format("%.3f", strafePower));
        telemetry.addData("Rotate Power", String.format("%.3f", rotatePower));
        
        // Apply movement
        setMotorPowers(forwardPower, strafePower, rotatePower);
        
        // Action description
        if (Math.abs(distanceError) > POSITION_TOLERANCE) {
            telemetry.addData("Action", distanceError > 0 ? "Moving forward (too far)" : "Moving backward (too close)");
        }
        if (Math.abs(angleError) > ANGLE_TOLERANCE) {
            telemetry.addData("Action", "Rotating to face tag");
        }
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
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }
        
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
    
    private void handlePoseLoss(long currentTime) {
        long timeSinceLastPose = currentTime - lastValidPoseTime;
        
        telemetry.addData("--- Pose Loss Handling ---", "");
        telemetry.addData("Time since last pose", String.format("%d ms", timeSinceLastPose));
        telemetry.addData("Tag recently detected", tagRecentlyDetected);
        
        if (tagRecentlyDetected && timeSinceLastPose < POSE_TIMEOUT_MS && lastValidPose != null) {
            // Continue with last known pose for a short time
            telemetry.addData("Using last known pose", "Continuing navigation");
            telemetry.addData("Last pose age", String.format("%d ms", timeSinceLastPose));
            performNavigation(lastValidPose);
        } else {
            // Stop motors if pose has been lost for too long
            telemetry.addData("Pose lost too long", "Stopping motors");
            telemetry.addData("Action", "Waiting for AprilTag to come back into view");
            stopAllMotors();
            tagRecentlyDetected = false;
        }
    }
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
