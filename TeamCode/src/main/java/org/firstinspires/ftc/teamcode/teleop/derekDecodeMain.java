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

// CONTROLS:
/*
Move: Left stick up/down/left/right
Turn: Right stick left/right
Reset heading: Options (front is the way it shoots)
Intake: Right bumper (r1)
Intake reversed: Left bumper (l1)
Move indexer: b (circle)
Move elevator up: dpad_up
Move elevator down: dpad_down
Auto aim to goal: left trigger (l2)
Run shooter: right trigger(r2)
 */

@Config
@Configurable
@TeleOp(name = "Derek Decode TeleOp")
public class derekDecodeMain extends LinearOpMode {

    public static double flyWheelPower = 1400;
    //    public static double intakePower = 0.75;
    public static double spindexerPower = 0; // Continuous spinning servo

    public static double indexerEngage = 0.86; // Servo mode middle gear servo
    public static double indexerDisengage = 0.7;
    public static double indexerPos = indexerEngage;
    //    public static double elevatorPower = 1;
    public static double elevatorDown = 0.84;
    public static double elevatorUp = 0.3;
    public static double elevatorPos = elevatorDown;

    public static double spindexerRed = 75;
    public static boolean transferReady = false;

    public static double intakePower = 0;

    public static boolean spindexerActive = false;

//    final double indexerHome = 0.93;
//    final double indexerPush1 = 0.85;
//    final double indexerPush2 = 0.80;
//    final double indexerPush3 = 0.75;

//    final double indexerTransfer = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        CRServo spindexer = hardwareMap.crservo.get("spindexer");
        Servo indexer = hardwareMap.servo.get("indexer");
        ColorSensor spindexerColor = hardwareMap.colorSensor.get("spindexerColor");

        Servo elevator = hardwareMap.servo.get("elevator");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");


        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0);

        limelight.stop();


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();



        if (isStopRequested()) return;



        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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

            if (gamepad1.right_bumper) {
                intakePower = 1;
                indexerPos = indexerDisengage;
            } else if (gamepad1.left_bumper) {
                intakePower = -0.7;
            } else {
                intakePower = 0;
            }

            if (gamepad1.right_trigger >= 0.2) {
                double flywheelpow = ll.fetchFlywheelSpeed(limelight);
                flywheel1.setVelocity(flywheelpow);
                flywheel2.setVelocity(flywheelpow);
                if (flywheelpow - flywheel1.getVelocity() < 100 && transferReady) elevatorPos = elevatorUp;
            } else {
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            }

            if (gamepad1.dpad_up) {
                elevatorPos = elevatorUp;
            } else if (gamepad1.dpad_down) {
                elevatorPos = elevatorDown;
            }

            if (gamepad1.b) spindexerActive = true;
            if (spindexerActive) {
                spindexerPower = -0.5;
                if (spindexerColor.red() > spindexerRed) {
                    spindexerPower = 0;
                    transferReady = true;
                }
            }

            intake.setPower(intakePower);
            spindexer.setPower(spindexerPower);
            indexer.setPosition(indexerPos);
            elevator.setPosition(elevatorPos);

            if (gamepad1.left_trigger >= 0.2) { // While loop unnecessary if left trigger is held
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    telemetry.addData("Limelight ID", fiducial.getFiducialId());
                }
                double horizontalOffset = result.getTx();
                double turnPower = 0.25;
                double tolerance = 1;

//                while (opModeIsActive() && Math.abs(horizontalOffset) > tolerance) { // While loop unnecessary if left trigger is held
                if (Math.abs(horizontalOffset) > tolerance) {
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
//                    LLResult newresult = limelight.getLatestResult();
//                    horizontalOffset = newresult.getTx();
                }
//                frontRightMotor.setPower(0);
//                frontLeftMotor.setPower(0);
//                backLeftMotor.setPower(0);
//                backRightMotor.setPower(0);
            }
            telemetry.addData("Red: ", + spindexerColor.red());
            telemetry.addData("Blue: ", + spindexerColor.blue());
            telemetry.addData("Green: ", + spindexerColor.green());
            telemetry.addData("Alpha: ", + spindexerColor.alpha());
            telemetry.update();
        }
    }
}
