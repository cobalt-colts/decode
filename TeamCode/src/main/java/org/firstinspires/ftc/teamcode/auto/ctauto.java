package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// WORKS AND I HAVE NO F**KING IDEA WHY

@Config
@Configurable
@Autonomous(name = "Far Red-CT (3 + 0)")
public class ctauto extends LinearOpMode {

    private void strafe(boolean isright, double power) {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        if (isright) {
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
        } else if (!isright) {
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
        }
    }

    public static int waittime = 1000;
    public static double flywheelpower = 1400;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        Servo indexer = hardwareMap.servo.get("indexer");

        DcMotor elevator = hardwareMap.dcMotor.get("elevator");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        waitForStart();

        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(-0.5);

        Thread.sleep(1000);

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        flywheel.setVelocity(flywheelpower);
        elevator.setPower(1);


        indexer.setPosition(0.75);

        Thread.sleep(5000);

        strafe(true, 0.5);
        Thread.sleep(1500);
        strafe(true,0);



//        Follower follower = Constants.createFollower(hardwareMap);
//        follower.activateAllPIDFs();
//        Path forwards = new Path(new BezierLine(new Pose(0,0), new Pose(35,0)));
//        follower.followPath(forwards);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            follower.update();

//            if (!follower.isBusy() && !pathDone) {
//                pathDone = true;
//            }

            telemetry.addLine("complete!");
            telemetry.update();
        }
    }
}