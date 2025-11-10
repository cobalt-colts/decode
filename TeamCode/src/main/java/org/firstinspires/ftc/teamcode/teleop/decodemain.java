package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ll;

import java.util.List;


@Config
@Configurable
@TeleOp(name = "Meet 1 TeleOp")
public class decodemain extends LinearOpMode {

    public static double flyWheelPower = 1400;
    public static double intakePower = 0.75;

    public static double indexerPos = 0.93;

    public static double liftPos = 1;

    public static boolean indexeractive = false;

    public static double engageopenpos = 0;

    final double indexerHome = 0.93;
    final double indexerPush1 = 0.85;
    final double indexerPush2 = 0.80;
    final double indexerPush3 = 0.75;

    final double indexerTransfer = 0.75;

    private boolean isWhite(ColorSensor sensor){
        if (sensor.red() >= 120 && sensor.blue() >= 120 && sensor.green() >= 120){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        ColorSensor indexsensor = hardwareMap.colorSensor.get("indexSensor");


        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0);

        limelight.stop();


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();



        if (isStopRequested()) return;



        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            limelight.start();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.cross) {
                intake.setPower(1);
                indexengage.setPosition(engageopenpos);
                indexerPos = indexerHome;
            } else if (gamepad1.square) {
                intake.setPower(-0.4);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.triangle) {
                indexengage.setPosition(1);

            }

            if (gamepad1.right_trigger >= 0.2) {
                double flywheelpow = ll.fetchFlywheelSpeed(limelight);
                telemetry.addData("flywheelvelocity", flyWheelPower);
                thrower1.setVelocity(flyWheelPower);
            } else {
                thrower1.setVelocity(0);
            }

            if (gamepad1.left_bumper){
                lift.setPosition(liftPos);
            } else {
                lift.setPosition(0);
            }


                    if (gamepad1.right_bumper) {
                      LLResult result = limelight.getLatestResult();
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        double horizontalOffset = result.getTx();
                        double turnPower = 0.25;
                        double tolerance = 1;

                        while (opModeIsActive() && Math.abs(horizontalOffset) > tolerance) {
                            if (horizontalOffset > 0) {
                                frontLeftMotor.setPower(-turnPower);
                                frontRightMotor.setPower(turnPower);
                                backLeftMotor.setPower(-turnPower);
                                backRightMotor.setPower(turnPower);

                            } else {
                                frontLeftMotor.setPower(turnPower);
                                frontRightMotor.setPower(-turnPower);
                                backLeftMotor.setPower(turnPower);
                                backLeftMotor.setPower(-turnPower);
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
        }
