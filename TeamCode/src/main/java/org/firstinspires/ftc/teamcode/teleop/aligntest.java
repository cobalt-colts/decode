package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList;

import java.util.List;

@TeleOp
public class aligntest extends LinearOpMode {

    private Limelight3A limelight;


    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();
        while (opModeIsActive()) {


            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();


            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("limelight x:", result.getTx());
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        telemetry.addData("Limelight ID", fiducial.getFiducialId());
                    }
                    if (gamepad1.right_bumper){
                        double horizontalOffset = result.getTx();
                        double turnPower = 0.25;
                        double tolerance = 1;

                        while (opModeIsActive() && Math.abs(horizontalOffset) > tolerance){
                            if (horizontalOffset > 0){
                                frontLeftMotor.setPower(turnPower);
                                frontRightMotor.setPower(-turnPower);
                                backLeftMotor.setPower(turnPower);
                                backRightMotor.setPower(-turnPower);

                            } else {
                                frontLeftMotor.setPower(-turnPower);
                                frontRightMotor.setPower(turnPower);
                                backLeftMotor.setPower(-turnPower);
                                backLeftMotor.setPower(turnPower);
                            }
                            LLResult newresult = limelight.getLatestResult();
                            horizontalOffset = newresult.getTx();
                        }



                        frontRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);

                    }
                }
                telemetry.update();
            }
            frontRightMotor.setPower(gamepad1.right_stick_x);
            frontLeftMotor.setPower(-gamepad1.right_stick_x);
            backLeftMotor.setPower(-gamepad1.right_stick_x);
            backRightMotor.setPower(gamepad1.right_stick_x);
        }

    }
}
