package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Test TeleOp that uses the MecanumDriveBaseOpMode for motor configuration testing.
 * 
 * This is a concrete implementation of the base class that provides:
 * - Standard drive controls with corrected coordinate mapping
 * - Comprehensive telemetry for testing motor configuration
 * 
 * Use this to verify that:
 * 1. Forward movement moves robot forward
 * 2. Left strafe moves robot left
 * 3. Right strafe moves robot right
 * 4. Left turn rotates robot counter-clockwise
 * 5. Right turn rotates robot clockwise
 */
@TeleOp(name = "Mecanum Drive Test", group = "Test")
public class MecanumDriveTestTeleOp extends MecanumDriveBaseOpMode {

    @Override
    protected void onInit() {
        // Add any custom initialization here
        telemetry.addData("Test Mode", "Motor Configuration Testing");
    }

    @Override
    protected void onLoop() {
        // Add any custom loop functionality here
        // For testing, we don't need additional functionality
    }

    @Override
    protected void getTelemetryData() {
        // Add any custom telemetry here
        telemetry.addData("Test Status", "All movements should work correctly");
    }
}