package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

@Config
@Configurable
@Autonomous(name = "Meet 1-RED Goal", preselectTeleOp = "Meet 1 TeleOp SAFE")
public class ctREDGoal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState = 0;
    private boolean indexon = false;

    // ---------- Path Definitions ----------
    public static class Paths {
        public static PathChain preload, offline, line1, line1launch, autoreturn;

        public Paths(Follower follower) {
            preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 128.000), new Pose(85.000, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(216))
                    .build();

            offline = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 100.000),
                                    new Pose(77.646, 81.667),
                                    new Pose(110.000, 80.500) // x 105
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(0))
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
                            new BezierCurve(
                                    new Pose(120.000, 83.368),
                                    new Pose(81.667, 82.595),
                                    new Pose(101.929, 93.577),
                                    new Pose(85.000, 100.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .build();
        }
    }

    public boolean isWhite(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        return c.red >= 0.04 && c.green >= 0.04 && c.blue >= 0.04;
    }

    // Blocking index routine for LinearOpMode
    public void index(NormalizedColorSensor sensor, CRServo indexer) {
        final double power = 0.075; // 0.125
        long start = System.currentTimeMillis();

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        intake.setPower(.5);
        indexer.setPower(power);

//        // Phase 1: wait for white
//        while (opModeIsActive() && !isWhite(sensor)) {
//            if (System.currentTimeMillis() - start > 3000) break;
//            sleep(10);
//        }

        // Phase 2: wait for NOT white
        start = System.currentTimeMillis();
        while (opModeIsActive() && isWhite(sensor)) {
            if (System.currentTimeMillis() - start > 3000) break;
            sleep(10);
        }

        // Phase 3: wait for white again
        start = System.currentTimeMillis();
        while (opModeIsActive() && !isWhite(sensor)) {
            if (System.currentTimeMillis() - start > 3000) break;
            sleep(10);
        }

        indexer.setPower(0);
        intake.setPower(0);
    }

    public void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- Timers -----
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // ----- Follower -----
        follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        new Paths(follower);

        follower.setStartingPose(new Pose(120.000, 128.000, Math.toRadians(216.5)));

        // ----- Hardware -----
        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        NormalizedColorSensor indexsensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");

        // Shooter setup
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        thrower1.setVelocityPIDFCoefficients(
                ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(
                ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);

        limelight.pipelineSwitch(0);
        limelight.stop();

        telemetry.addLine("Initialized - Waiting for Start");
        telemetry.update();

        hood.setPosition(0.4);

        waitForStart();
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {

            follower.update();

            switch (pathState) {

                case 0:
                    thrower1.setVelocity(2000 * ShooterPIDConfig.TICKS_PER_REV / 60.0);
                    thrower2.setVelocity(2000 * ShooterPIDConfig.TICKS_PER_REV / 60.0);

                    indexengage.setPosition(0.825);
                    follower.followPath(Paths.preload, true);
                    setPathState(1);
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        double speed = ll.fetchFlywheelSpeed(limelight) *
                                ShooterPIDConfig.TICKS_PER_REV / 60.0;

                        thrower1.setVelocity(speed);
                        thrower2.setVelocity(speed);

                        sleep(2000);
                        lift.setPosition(0);
                        sleep(2000);
                        lift.setPosition(0.9);
                        for (int i = 0; i < 2; i++) {
                            sleep(1000);
                            index(indexsensor, indexer);
                            lift.setPosition(0);
                            sleep(2000);
                            lift.setPosition(0.9);
                        }

                        indexengage.setPosition(0.7);
                        intake.setPower(1);
                        follower.followPath(Paths.offline, 0.8, false);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!follower.isBusy())
                        setPathState(-1);
                    break;

                default:
                    break;
            }

            telemetry.addData("State", pathState);
            telemetry.update();
        }
    }
}
