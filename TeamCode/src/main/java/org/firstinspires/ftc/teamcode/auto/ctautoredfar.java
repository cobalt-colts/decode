package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// WORKS AND I HAVE NO F**KING IDEA WHY

@Config
@Configurable
@Autonomous(name = "Far Red-CT (3 + 0)")
public class ctautoredfar extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static class Paths {

        public static PathChain preload;
        public static PathChain offline;
        public static PathChain line1;
        public static PathChain line1launch;

        public Paths(Follower follower) {
            preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 128.000), new Pose(80.000, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(216.5))
                    .build();

            offline = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(80.000, 97), new Pose(100.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(0))
                    .build();

            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97.000, 83.500), new Pose(120.000, 83.368))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line1launch = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 83.368), new Pose(80.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(216.5))
                    .build();
        }
    }


    public void autonomousPathUpdate() throws InterruptedException {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        Servo indexer = hardwareMap.servo.get("indexer");

        DcMotor elevator = hardwareMap.dcMotor.get("elevator");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        switch (pathState) {
            case 0:
                follower.followPath(Paths.preload, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    flywheel.setVelocity(-flywheelpower);
                    elevator.setPower(1);

                    indexer.setPosition(0.75);

                    Thread.sleep(5000);

                    flywheel.setVelocity(0);
                    elevator.setPower(0);

                    indexer.setPosition(0.93);

                    follower.followPath(Paths.offline,false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    Thread.sleep(500);
                    follower.followPath(Paths.line1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Paths.line1launch, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    flywheel.setVelocity(-flywheelpower);
                    elevator.setPower(1);

                    indexer.setPosition(0.75);

                    Thread.sleep(5000);

                    intake.setPower(0);
                    flywheel.setVelocity(0);
                    elevator.setPower(0);

                    follower.followPath(Paths.offline, true);
                    setPathState(-1);
                }
        }
    }

    // These change the states of the paths and actions. It will also reset the timers of the individual switches
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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
    public static double flywheelpower = 1430;

    boolean pathDone = false;



    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        Paths paths = new Paths(follower);
        follower.setStartingPose(new Pose(120.000, 128.000, Math.toRadians(216.5)));


    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {}
    @Override
    public void init_loop() {}
}