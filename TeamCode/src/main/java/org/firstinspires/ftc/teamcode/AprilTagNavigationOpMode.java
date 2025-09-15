package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    private static final double ROTATION_SPEED = 0.3; // Speed for turning when tag not in view
    private static final double MOVEMENT_SPEED = 0.4; // Speed for moving towards tag
    private static final double POSITION_TOLERANCE = 2.0; // Tolerance in inches
    private static final double ANGLE_TOLERANCE = 5.0; // Tolerance in degrees
    
    // State variables
    private boolean navigationActive = false;
    private boolean lastButtonState = false;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Set limelight to pipeline 9 (configured for AprilTag IDs 20-24)
        limelight.pipelineSwitch(9);

        telemetry.addData("Hardware initialized", "Ready");
        telemetry.addData("Press A button to start navigation to AprilTag ID", TARGET_APRILTAG_ID);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Start limelight
        limelight.start();
        telemetry.addData("Limelight started", "Pipeline 9 active");
        telemetry.update();

        while (opModeIsActive()) {
            // Check for button press to start/stop navigation
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                // Button just pressed
                navigationActive = !navigationActive;
                if (navigationActive) {
                    telemetry.addData("Navigation", "STARTED - Looking for AprilTag ID " + TARGET_APRILTAG_ID);
                } else {
                    telemetry.addData("Navigation", "STOPPED");
                    stopAllMotors();
                }
            }
            lastButtonState = currentButtonState;

            if (navigationActive) {
                performAprilTagNavigation();
            } else {
                // Manual control when navigation is not active
                performManualControl();
            }

            telemetry.update();
        }
    }

    private void performAprilTagNavigation() {
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            
            // Look for our target AprilTag ID
            LLResultTypes.FiducialResult targetTag = null;
            for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                    targetTag = fiducial;
                    break;
                }
            }

            if (targetTag != null) {
                // Tag found! Navigate to it
                navigateToTag(targetTag);
            } else {
                // Tag not in view, rotate slowly to find it
                telemetry.addData("Status", "AprilTag ID " + TARGET_APRILTAG_ID + " not in view - rotating");
                rotateToFindTag();
            }
        } else {
            // No valid result, rotate to find tag
            telemetry.addData("Status", "No valid limelight data - rotating");
            rotateToFindTag();
        }
    }

    private void navigateToTag(LLResultTypes.FiducialResult tag) {
        // Get tag position data from the main result (not individual fiducial)
        LLResult result = limelight.getLatestResult();
        double tx = result.getTx(); // Horizontal offset from center (-29.8 to 29.8 degrees)
        double ty = result.getTy(); // Vertical offset from center (-24.85 to 24.85 degrees)
        double ta = result.getTa(); // Target area (0-100% of image)
        
        telemetry.addData("Tag Found", "ID: " + tag.getFiducialId());
        telemetry.addData("TX (horizontal)", tx);
        telemetry.addData("TY (vertical)", ty);
        telemetry.addData("TA (area)", ta);

        // Calculate distance from tag (rough estimation based on area)
        // This is a simplified calculation - you may need to calibrate based on your setup
        double estimatedDistance = calculateDistanceFromArea(ta);
        
        telemetry.addData("Estimated Distance", String.format("%.1f inches", estimatedDistance));

        // Calculate movement needed
        double distanceError = estimatedDistance - TARGET_DISTANCE_INCHES;
        double angleError = tx; // Horizontal offset is our angle error

        // Check if we're close enough to target position
        if (Math.abs(distanceError) < POSITION_TOLERANCE && Math.abs(angleError) < ANGLE_TOLERANCE) {
            telemetry.addData("Status", "ARRIVED at target position!");
            stopAllMotors();
            return;
        }

        // Calculate motor powers for movement
        double forwardPower = 0;
        double strafePower = 0;
        double rotatePower = 0;

        // Forward/backward movement based on distance
        if (Math.abs(distanceError) > POSITION_TOLERANCE) {
            forwardPower = Math.max(-MOVEMENT_SPEED, Math.min(MOVEMENT_SPEED, 
                -distanceError / TARGET_DISTANCE_INCHES * MOVEMENT_SPEED));
        }

        // Rotation to center the tag
        if (Math.abs(angleError) > ANGLE_TOLERANCE) {
            rotatePower = Math.max(-MOVEMENT_SPEED, Math.min(MOVEMENT_SPEED, 
                -angleError / 30.0 * MOVEMENT_SPEED));
        }

        // Apply movement
        setMotorPowers(forwardPower, strafePower, rotatePower);
        
        telemetry.addData("Movement", String.format("Forward: %.2f, Rotate: %.2f", forwardPower, rotatePower));
    }

    private void rotateToFindTag() {
        // Rotate slowly to find the tag
        setMotorPowers(0, 0, ROTATION_SPEED);
    }

    private double calculateDistanceFromArea(double area) {
        // This is a simplified distance calculation based on tag area
        // You may need to calibrate these values for your specific setup
        // Generally, larger area = closer distance
        if (area > 0) {
            // Rough estimation: area of 10% corresponds to about 3 feet
            // This is a very rough approximation and should be calibrated
            return Math.sqrt(100.0 / area) * 12.0; // Convert to inches
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

    private void performManualControl() {
        // Manual control when navigation is not active
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Manual Control", "Active - Use left stick to drive, right stick X to rotate");
    }
}
