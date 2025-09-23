package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Example TeleOp that extends MecanumDriveBaseOpMode to show how to add additional functionality.
 * 
 * This example shows how to:
 * - Add additional hardware (servo)
 * - Override drive controls for custom behavior
 * - Add custom telemetry
 * - Add additional gamepad controls
 */
@TeleOp(name = "Example Mecanum TeleOp", group = "Example")
public class ExampleMecanumTeleOp extends MecanumDriveBaseOpMode {

    private Servo exampleServo;
    private double servoPosition = 0.5;

    @Override
    protected void onInit() {
        // Initialize additional hardware
        exampleServo = hardwareMap.get(Servo.class, "example_servo");
        exampleServo.setPosition(servoPosition);
        
        telemetry.addData("Example Mode", "Extended Mecanum Drive with Servo");
    }

    @Override
    protected void onLoop() {
        // Handle additional gamepad controls
        if (gamepad1.a) {
            servoPosition = 0.0; // Move servo to position 0
        } else if (gamepad1.b) {
            servoPosition = 1.0; // Move servo to position 1
        } else if (gamepad1.x) {
            servoPosition = 0.5; // Move servo to center
        }
        
        // Update servo position
        exampleServo.setPosition(servoPosition);
    }

    @Override
    protected void getTelemetryData() {
        // Add custom telemetry
        telemetry.addData("Servo Position", "%.2f", servoPosition);
        telemetry.addData("Controls", "A=0, B=1, X=0.5");
    }

    // Optional: Override drive controls for custom behavior
    @Override
    protected void handleDriveControls() {
        // You can override this method to implement custom drive controls
        // For example, field-relative driving, different speed scaling, etc.
        
        // For this example, we'll use the standard controls
        super.handleDriveControls();
        
        // But we could add custom modifications here, like:
        // - Different speed scaling
        // - Field-relative controls
        // - Custom button combinations
    }
}
