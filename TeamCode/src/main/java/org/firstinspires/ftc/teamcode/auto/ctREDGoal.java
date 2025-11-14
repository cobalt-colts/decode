package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

// WORKS AND I HAVE NO F**KING IDEA WHY

@Config
@Configurable
@Autonomous(name = "Far Red-CT (3 + 0)")
public class ctREDGoal extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int lastPathState;
    private int indexerState = 0;
    private boolean indexon = false;

    public static class Paths {

        public static PathChain preload;
        public static PathChain offline;
        public static PathChain line1;
        public static PathChain line1launch;
        public static PathChain autoreturn;


        public Paths(Follower follower) {
            preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 128.000), new Pose(71.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(225))
                    .build();

            offline = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(71.000, 80.000), new Pose(100.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.000, 83.500), new Pose(120.000, 83.368))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line1launch = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 83.368), new Pose(71.000, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();
        }
    }

    public boolean isWhite(NormalizedColorSensor sensor){
        NormalizedRGBA colors = sensor.getNormalizedColors();
        return colors.red >= 0.04 && colors.blue >= 0.04 && colors.green >= 0.04;

    }

    // Replace index state-machine with a blocking single-run index() call.
    public static int indexTimeoutMs = 3000; // configurable timeout per phase in ms

    public void index() throws InterruptedException {
        // Blocking single-index cycle:
        // 1) spin until first white detected
        // 2) continue until white is lost
        // 3) continue until next white detected
        // then stop and return.

        NormalizedColorSensor indexsensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");
        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        final double power = 0.125;
        final long overallStart = System.currentTimeMillis();

        // Safety: overall timeout to avoid infinite blocking
        final long overallTimeout = indexTimeoutMs * 4L;

        // Helper to check overall timeout
        java.util.function.BooleanSupplier timedOut = () -> (System.currentTimeMillis() - overallStart) > overallTimeout;

        // Phase 1: run until first white detected
        indexer.setPower(power);
        while (!isWhite(indexsensor) && !timedOut.getAsBoolean()) {
            Thread.sleep(10);
        }
        if (timedOut.getAsBoolean()) {
            indexer.setPower(0);
            indexerState = 0;
            indexon = false;
            return;
        }

        // Phase 2: wait until white is no longer detected
        long phaseStart = System.currentTimeMillis();
        while (isWhite(indexsensor) && (System.currentTimeMillis() - phaseStart) < indexTimeoutMs) {
            Thread.sleep(10);
        }
        // Phase 3: run until next white detected (the disk has been moved into position)
        phaseStart = System.currentTimeMillis();
        while (!isWhite(indexsensor) && (System.currentTimeMillis() - phaseStart) < indexTimeoutMs && !timedOut.getAsBoolean()) {
            Thread.sleep(10);
        }

        // Stop motor and reset flags
        indexer.setPower(0);
        indexerState = 0;
        indexon = false;
    }

    public void autonomousPathUpdate() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF
        );

        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);


        DcMotor intake = hardwareMap.dcMotor.get("intake");
        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        NormalizedColorSensor indexsensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");


        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0);

        limelight.stop();


        switch (pathState) {
            case 0:
                thrower1.setVelocity(2000*ShooterPIDConfig.TICKS_PER_REV / 60.0);
                thrower2.setVelocity(2000*ShooterPIDConfig.TICKS_PER_REV / 60.0);
                indexengage.setPosition(0.825);
                follower.followPath(Paths.preload, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    thrower1.setVelocity(ll.fetchFlywheelSpeed(limelight)* ShooterPIDConfig.TICKS_PER_REV / 60.0);
                    thrower2.setVelocity(ll.fetchFlywheelSpeed(limelight)* ShooterPIDConfig.TICKS_PER_REV / 60.0);
                    lift.setPosition(0.9);
                    Thread.sleep(1000);
                    lift.setPosition(0);
                    index();
                    lift.setPosition(0.9);
                    Thread.sleep(1000);
                    lift.setPosition(0);
                    index();
                    lift.setPosition(0.9);
                    Thread.sleep(1000);
                    lift.setPosition(0);
                    indexengage.setPosition(0.7);
                    follower.followPath(Paths.offline,false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    setPathState(-1);
//                    intake.setPower(1);
//                    Thread.sleep(500);
//                    follower.followPath(Paths.line1, true);
//                    setPathState(3);
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
//                    flywheel.setVelocity(-1330);
//                    elevator.setPower(1);
//
//                    indexer.setPosition(0.75);
//
//                    Thread.sleep(2500);
//
//                    flywheel.setVelocity(-1330*0.8);
//
//                    Thread.sleep(2500);
//
//                    intake.setPower(0);
//                    flywheel.setVelocity(0);
//                    elevator.setPower(0);

                    follower.followPath(Paths.autoreturn, true);
                    setPathState(-1);
                }
            case 5:
                if(!indexon){

                    break;
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
    public static double flywheelpower = 1400;
    public static int indexTimeout = 3000; // ...existing code...

    boolean pathDone = false;



    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
            index();
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