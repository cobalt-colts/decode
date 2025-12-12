package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//DOESN'T WORK AND I HAVE NO F**KING IDEA WHY

@Disabled
@Configurable
@Autonomous
public class r3hauto extends LinearOpMode {
//    public static Follower follower;

    DcMotor flywheel = hardwareMap.dcMotor.get("flywheel");
    CRServo elevator = hardwareMap.crservo.get("elevator");
    Servo indexer = hardwareMap.servo.get("indexer");


    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

    boolean pathDone = false;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

//        follower = Constants.createFollower(hardwareMap);
//        follower.activateAllPIDFs();
//        Path forwards = new Path(new BezierLine(new Pose(0,0), new Pose(35,0)));
//        follower.followPath(forwards);

        indexer.setPosition(0.75);

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        Thread.sleep(1000);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        while (opModeIsActive()) {

//            follower.update();

//            if (!follower.isBusy() && !pathDone) {
//                pathDone = true;
//            }

            if (pathDone) {
                flywheel.setPower(0.85);
                elevator.setPower(1);
            }
        }
    }

}
