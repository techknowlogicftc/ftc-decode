package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Base class for OpModes that use SampleMecanumDrive with corrected coordinate mapping.
 * 
 * This class provides:
 * - Corrected coordinate mapping for proper robot movement
 * - Standard gamepad controls (left stick for movement, right stick X for turning)
 * - Boost mode (right trigger for 2x speed)
 * - Optional telemetry display
 * 
 * Controls:
 * - Left stick Y: Forward/Backward
 * - Left stick X: Strafe Left/Right  
 * - Right stick X: Turn Left/Right
 * - Right trigger: Boost mode (2x speed)
 * 
 * To use this as a base class:
 * 1. Extend this class instead of LinearOpMode
 * 2. Override onInit() for initialization
 * 3. Override onLoop() for additional functionality
 * 4. Override getTelemetryData() to add custom telemetry
 */
public abstract class MecanumDriveBaseOpMode extends LinearOpMode {

    protected SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        // Initialize the drive system
        drive = new SampleMecanumDrive(hardwareMap);
        
        // Call subclass initialization
        onInit();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use left stick for movement, right stick X for turning");
        telemetry.addData("Boost", "Hold right trigger for 2x speed");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Handle drive controls
            handleDriveControls();
            
            // Call subclass loop
            onLoop();
            
            // Update the drive system
            drive.update();
            
            // Display telemetry
            displayTelemetry();
        }
    }

    /**
     * Handles the standard drive controls with corrected coordinate mapping.
     * This method can be overridden if custom drive controls are needed.
     */
    protected void handleDriveControls() {
        // Get gamepad inputs
        double leftStickY = -gamepad1.left_stick_y;  // Forward/Backward (inverted for natural feel)
        double leftStickX = gamepad1.left_stick_x;   // Strafe Left/Right
        double rightStickX = gamepad1.right_stick_x; // Turn Left/Right
        double boost = gamepad1.right_trigger;       // Boost mode
        
        // Apply boost multiplier (1.0 to 2.0)
        double speedMultiplier = 1.0 + boost;
        
        // Scale the inputs
        leftStickY *= speedMultiplier;
        leftStickX *= speedMultiplier;
        rightStickX *= speedMultiplier;
        
        // Clamp values to valid range
        leftStickY = Range.clip(leftStickY, -1.0, 1.0);
        leftStickX = Range.clip(leftStickX, -1.0, 1.0);
        rightStickX = Range.clip(rightStickX, -1.0, 1.0);
        
        // Set the drive power using the corrected coordinate mapping
        drive.setWeightedDrivePower(
            new com.acmerobotics.roadrunner.geometry.Pose2d(
                leftStickY,      // X (forward/backward)
                -leftStickX,     // Y (strafe) - inverted for correct direction
                -rightStickX     // Heading (turn) - inverted for correct direction
            )
        );
    }

    /**
     * Displays standard telemetry. Can be overridden to customize telemetry display.
     */
    protected void displayTelemetry() {
        // Get current gamepad inputs for telemetry
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;
        double boost = gamepad1.right_trigger;
        double speedMultiplier = 1.0 + boost;
        
        // Standard drive telemetry
        telemetry.addData("Left Stick Y", "%.2f (Forward/Backward)", leftStickY);
        telemetry.addData("Left Stick X", "%.2f (Strafe)", leftStickX);
        telemetry.addData("Right Stick X", "%.2f (Turn)", rightStickX);
        telemetry.addData("Boost", "%.2f (Speed: %.1fx)", boost, speedMultiplier);
        telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f°", 
            drive.getPoseEstimate().getX(),
            drive.getPoseEstimate().getY(),
            Math.toDegrees(drive.getPoseEstimate().getHeading()));
        
        // Motor position telemetry
        telemetry.addData("Motor Positions", "");
        telemetry.addData("  Front Left", "%.2f", drive.getWheelPositions().get(0));
        telemetry.addData("  Back Left", "%.2f", drive.getWheelPositions().get(1));
        telemetry.addData("  Back Right", "%.2f", drive.getWheelPositions().get(2));
        telemetry.addData("  Front Right", "%.2f", drive.getWheelPositions().get(3));
        
        // Call subclass telemetry
        getTelemetryData();
        
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    // Abstract methods for subclasses to implement

    /**
     * Called once during initialization. Override this method to add custom initialization.
     */
    protected abstract void onInit();

    /**
     * Called every loop iteration. Override this method to add custom functionality.
     */
    protected abstract void onLoop();

    /**
     * Called during telemetry display. Override this method to add custom telemetry data.
     */
    protected abstract void getTelemetryData();
}
