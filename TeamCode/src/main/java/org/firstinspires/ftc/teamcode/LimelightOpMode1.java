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
public class LimelightOpMode1 extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("runOpMode() START", 0);
        telemetry.update();

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontleft");//rear right
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backleft"); //front right
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontright");//slider
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backright");//lift
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Detected all hardware: ", true);
        telemetry.update();

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(9);

        waitForStart();

        telemetry.addData("After waitForStart() ", true);
        telemetry.update();

        if (isStopRequested()) return;

        /*
         * Starts polling for data.
         */
        limelight.start();

        telemetry.addData("limelight started ", true);
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("opModeIsActive() START", 1);
            telemetry.update();
            LLResult result = limelight.getLatestResult();
            telemetry.addData("result: ", result);
            telemetry.update();
            if (result != null) {
                telemetry.addData("result not NULL ", true);
                telemetry.addData("result VALID: ", result.isValid());

                List<LLResultTypes.FiducialResult> fiducialResultList = result.getFiducialResults();
                telemetry.addData("fiducialResultList isEmpty(): ", fiducialResultList.isEmpty());

                if (!fiducialResultList.isEmpty()) {
                    LLResultTypes.FiducialResult fiducialResult0 = fiducialResultList.get(0);
                    if (fiducialResult0 != null) {
                        telemetry.addData("fiducialResult0: ", fiducialResult0);

                    }
                }

                
                if (result.isValid()) {

                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());

                }
            }

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.update();
        }
    }
}
