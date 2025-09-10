package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Configurable
@TeleOp(name = "Main Decode TeleOp")
public class decodemain extends LinearOpMode {

    public static double flyWheelPower = 0.85;
    public static double intakePower = 0.75;

    public static double indexerPos = 0.92;

    public static double elevatorPower = 1;

    public static boolean indexeractive = false;

    final double indexerHome = 0.92;
    final double indexerPush1 = 0.85;
    final double indexerPush2 = 0.80;
    final double indexerPush3 = 0.75;

    final double indexerTransfer = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor flywheel = hardwareMap.dcMotor.get("flywheel");

        Servo indexer = hardwareMap.servo.get("indexer");

        CRServo elevator = hardwareMap.crservo.get("elevator");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -(gamepad1.left_stick_y + gamepad2.left_stick_y); // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x + gamepad2.left_stick_x;
            double rx = -(gamepad1.right_stick_x + gamepad2.right_stick_x);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x - y;
            double rotY = x + y;

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            telemetry.addData("front left:", frontLeftPower);
            telemetry.addData("back left:", backLeftPower);
            telemetry.addData("front right:", frontRightPower);
            telemetry.addData("back right:", backRightPower);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }

            if (gamepad1.cross) {
                intake.setPower(1);
                indexerPos = indexerHome;
            } else if (gamepad1.square) {
                intake.setPower(-0.4);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_trigger >= 0.2) {
                flywheel.setPower(flyWheelPower);
            } else {
                flywheel.setPower(0);
            }

            if (gamepad1.right_trigger > gamepad1.left_trigger) {
                elevatorPower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > gamepad1.right_trigger) {
                elevatorPower = -gamepad1.left_trigger;
            } else {
                elevatorPower = 0;
            }

            if (gamepad1.dpad_down) indexerPos = indexerHome;
            if (gamepad1.dpad_right) indexerPos = indexerPush1;
            if (gamepad1.dpad_up) indexerPos = indexerPush2;
            if (gamepad1.dpad_left) indexerPos = indexerPush3;

            indexer.setPosition(indexerPos);
            elevator.setPower(elevatorPower);


            telemetry.update();
        }
    }
}