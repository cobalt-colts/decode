package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
@Configurable
@TeleOp
public class gpteleop extends LinearOpMode {

    public static PIDFController controller;

    public static double p = 0.005 ,i = 0 ,d = 0, f = 0.0004;

    public static double hoodpos = .15;

    public static int targetVelocity = 1300;

    private static DcMotorEx masterShootingSpeedMotor;
    private static DcMotorEx slaveShootingSpeedMotor;

    private static Servo hood;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        masterShootingSpeedMotor = hardwareMap.get(DcMotorEx.class , "thrower1");
        slaveShootingSpeedMotor = hardwareMap.get(DcMotorEx.class , "thrower2");

        hood = hardwareMap.servo.get("hood");

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        masterShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        slaveShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(p,i,d,f);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double currentVelocity = masterShootingSpeedMotor.getVelocity(); // /28)*60

            double pid = controller.calculate(currentVelocity , targetVelocity);

            double power = pid;

            if(targetVelocity < 100)
            {
                masterShootingSpeedMotor.setMotorDisable();
                slaveShootingSpeedMotor.setMotorDisable();
            }
            else {
                masterShootingSpeedMotor.setPower(power);
                slaveShootingSpeedMotor.setPower(power);
            }

            hood.setPosition(hoodpos);

        }
    }
}