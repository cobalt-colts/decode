package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.liftDown;
import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.liftUp;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorLimelight3A;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

import dev.nextftc.hardware.impl.ServoEx;

@Config
@Configurable
@Autonomous(name = "Meet 2-RED Goal", preselectTeleOp = "Meet 2.0 Teleop")
public class ctREDGoal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState = 0;
    private boolean indexon = false;
    private int indexState = 0;
    private int indexno = 0;
    private long indexStateTime = 0;

    private boolean mag1, mag2, mag3;

    // Track previous magnet states to detect transitions (magnet leaving prong)
    private boolean prevMag1 = false;
    private boolean prevMag2 = false;
    private boolean prevMag3 = false;

    // Track which initial magnets we've already counted (for pre-loaded balls)
    private boolean countedInitialMag1 = false;
    private boolean countedInitialMag2 = false;
    private boolean countedInitialMag3 = false;


    // Index variables from meet2teleop
    double indexPower = 0;
    public static double autoIndex = -0.1;
    public static double correctIndex = 0.1;
    public static double indexEngaged = 0.84;
    public static double indexDisengaged = 0.7;
    public static double intakePower = 0.5;

    private double llSpeed = 0;


    // ---------- Path Definitions ----------
    public static class Paths {

        public static PathChain launch1;
        public static PathChain line1;
        public static PathChain launch2;
        public static PathChain line2;

        public Paths(Follower follower) {
            launch1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.501, 124.975), new Pose(73.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(220))
                    .build();

            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 83.500), new Pose(104.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.000, 83.500), new Pose(125.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }

//
//    // Blocking index routine for LinearOpMode
//    public void index(CRServo indexer, DigitalChannel magnet1, DigitalChannel magnet2, DigitalChannel magnet3, Servo indexEngage, DcMotor intake) {
//        indexEngage.setPosition(indexEngaged);
//        intake.setPower(intakePower);
//
//        long timeoutStart = System.currentTimeMillis();
//        final long TIMEOUT = 5000;
//
//        while (opModeIsActive() && System.currentTimeMillis() - timeoutStart < TIMEOUT) {
//            boolean mag1State = !magnet1.getState();
//            boolean mag2State = !magnet2.getState();
//            boolean mag3State = !magnet3.getState();
//
//            if (mag3State) {
//                if (mag2State) {
//                    indexer.setPower(correctIndex);
//                } else if (mag1State) {
//                    indexer.setPower(0);
//                    break;
//                } else {
//                    indexer.setPower(autoIndex);
//                }
//            } else {
//                indexer.setPower(autoIndex);
//            }
//
//            sleep(50);
//        }
//
//        indexer.setPower(0);
//        intake.setPower(0);
//    }

    // Non-blocking index routine - tracks magnet transitions to detect ball movement
    public boolean updateIndex(
            CRServo indexer,
            DigitalChannel magnet1,
            DigitalChannel magnet2,
            DigitalChannel magnet3,
            Servo indexEngage,
            DcMotor intake
    ) {
        // read current (logical) magnet states: true == magnet present on prong
        boolean mag1State = !magnet1.getState();
        boolean mag2State = !magnet2.getState();
        boolean mag3State = !magnet3.getState();

        telemetry.addData("mag 1 state", mag1State);
        telemetry.addData("mag 2 state", mag2State);
        telemetry.addData("mag 3 state", mag3State);

        telemetry.update();

        Servo lift = hardwareMap.servo.get("lift");

        final long TIMEOUT = 5000;
        long now = System.currentTimeMillis();

        indexer.setPower(indexPower);

        switch (indexState) {

            // -------------------- START --------------------
            case 0:
                // engage and start intake/indexer
                indexEngage.setPosition(indexEngaged);
                intake.setPower(intakePower);
                indexPower = autoIndex;
                indexer.setPower(indexPower);
//                sleep(1500); // Sleep so we advance past any magnets from a previous cycle

                indexStateTime = now;
                indexState = 1;
                return false;


            // -------------------- INDEXING - WAIT FOR MAGNET TRANSITION --------------------
            case 1:
                // Detect when a magnet LEAVES (falling edge: was true, now false)
                // This indicates a ball has cycled past the sensor

//                indexPower = autoIndex;

                // Just go for it. When mag3&mag1, launch. It's fast enough that mag2 won't be lit yet
                if (mag3State && mag1State) {
//                    indexer.setPower(indexPower);
                    lift.setPosition(liftUp);
                    sleep(200);
                    lift.setPosition(liftDown);
                    indexno++;
                    if (indexno >= 4) {
                        // finished all 3 balls
                        indexPower = 0;
                        indexer.setPower(indexPower);
                        indexState = 67;
                        return true;
                    }
                }
//                if (mag3State){
//                    if (mag2State) {
//                        indexPower = correctIndex;  // Went too far, go back slowly
//                        indexer.setPower(indexPower);
//                    }
//                    else if (mag1State) {
//                        // At mag3 + mag1 + !mag2 = ready to launch:
//                        indexPower = 0;
//                        indexer.setPower(indexPower);
//                        lift.setPosition(liftUp);
//                        sleep(750);
//                        lift.setPosition(liftDown);
//                        indexState = 2;
//                        indexno++;
//                    }
//                } else {
//                    indexPower = autoIndex;
//                }
            return false;


            // -------------------- FINISHED ONE BALL - SETTLE & PREPARE FOR NEXT --------------------
            case 2:

                // turn off intake/indexer briefly to settle
                indexer.setPower(0);
                intake.setPower(0);

                if (indexno >= 3) {
                    // finished all 3 balls
                    indexState = 67;
                    return true;
                }

                // not finished → start another index cycle
                indexState = 0;
                return false;
        }

        return false;
    }


    public void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DigitalChannel magnet1 = hardwareMap.get(DigitalChannel.class, "mag1");
        DigitalChannel magnet2 = hardwareMap.get(DigitalChannel.class, "mag2");
        DigitalChannel magnet3 = hardwareMap.get(DigitalChannel.class, "mag3");

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
        indexengage.setPosition(0.825);

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        NormalizedColorSensor indexsensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");

        // Shooter setup
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setDirection(DcMotorSimple.Direction.REVERSE);
        thrower2.setDirection(DcMotorSimple.Direction.REVERSE);
        thrower1.setVelocityPIDFCoefficients(
                ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(
                ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);

        limelight.pipelineSwitch(0);
        limelight.stop();

        telemetry.addLine("Initialized - Waiting for Start");
        telemetry.update();

//        hood.setPosition(0.4); // 0.25

        waitForStart();
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {

            double throwervelocity = thrower1.getVelocity();

            mag1 = !magnet1.getState();
            mag2 = !magnet2.getState();
            mag3 = !magnet3.getState();

            follower.update();

            switch (pathState) {

                case 0:
                        // No balls loaded, launch and then index
                        thrower1.setVelocity(2000 * ShooterPIDConfig.TICKS_PER_REV / 60.0);
                        thrower2.setVelocity(2000 * ShooterPIDConfig.TICKS_PER_REV / 60.0);

                        indexengage.setPosition(0.84);

// Glenn 12/6/2025 so we don't drive
                        follower.followPath(Paths.launch1, true);
                        setPathState(1);
                    break;

                case 1:
                    if (!follower.isBusy() && throwervelocity >= 600) {

                        llSpeed = ll.fetchFlywheelSpeed(limelight) *
                                ShooterPIDConfig.TICKS_PER_REV / 60.0;

                        thrower1.setVelocity(llSpeed);
                        thrower2.setVelocity(llSpeed);

//                        indexno = 0;        // IMPORTANT: reset to 0
//                        indexState = 0;     // start index routine
                        setPathState(2);
                    }
                    break;


                case 2:
                    llSpeed = ll.fetchFlywheelSpeed(limelight) *
                            ShooterPIDConfig.TICKS_PER_REV / 60.0;

                    thrower1.setVelocity(llSpeed);
                    thrower2.setVelocity(llSpeed);

                    if (updateIndex(indexer, magnet1, magnet2, magnet3, indexengage, intake)) {
                     follower.followPath(Paths.line1, true);
                        setPathState(3);
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        setPathState(-1);
                    }
                    break;
            }

            telemetry.addData("Index Power", indexPower);
            telemetry.addData("State", pathState);
            telemetry.addData("Index State", indexState);
            telemetry.addData("indexno", indexno);
            telemetry.addData("llSpeed", llSpeed);
            telemetry.addData("Thrower Velocity", throwervelocity);
            telemetry.update();
        }
    }
}
