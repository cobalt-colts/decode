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
@Autonomous(name = "Meet 1-BLUE Far", preselectTeleOp = "Meet 1 TeleOp")
public class clBlueFar extends LinearOpMode {


    private int pathState = 1;
    public boolean isWhite(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        return c.red >= 0.04 && c.green >= 0.04 && c.blue >= 0.04;
    }

    // Blocking index routine for LinearOpMode
    public void index(NormalizedColorSensor sensor, CRServo indexer) {
        final double power = 0.085; // 0.125
        long start = System.currentTimeMillis();
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        intake.setPower(-.5);
        indexer.setPower(power);

//        // Phase 1: wait for white
//        while (opModeIsActive() && !isWhite(sensor)) {
//            if (System.currentTimeMillis() - start > 3000) break;
//            sleep(10);
//        }

        // Phase 2: wait for NOT white
        start = System.currentTimeMillis();
        while (opModeIsActive() && isWhite(sensor)) {
            if (System.currentTimeMillis() - start > 2500) {
                indexer.setPower(-0.25);
                if (System.currentTimeMillis() - start > 3000) break;
            }
            sleep(75); // 10
        }

        // Phase 3: wait for white again
        start = System.currentTimeMillis();
        while (opModeIsActive() && !isWhite(sensor)) {
            if (System.currentTimeMillis() - start > 2500) {
                indexer.setPower(-0.25);
                if (System.currentTimeMillis() - start > 3000) break;
            }
            sleep(75); // 10
        }

        indexer.setPower(0);
        intake.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- Hardware -----
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");
        hood.setPosition(0.2);

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

        hood.setPosition(0.2);
        indexengage.setPosition(0.825);

        waitForStart();

        while (opModeIsActive()) {
            double speed = 2150 * ShooterPIDConfig.TICKS_PER_REV / 60.0; // 2300   2200

            thrower1.setVelocity(speed);
            thrower2.setVelocity(speed);
            hood.setPosition(0.2); // 0.125?

            sleep(3500);
            for (int i = 0; i < 5; i++) {
                lift.setPosition(0);
                sleep(2000);
                lift.setPosition(0.9);
                sleep(1000);
                index(indexsensor, indexer);
            }

            backLeftMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backRightMotor.setPower(-0.5);
            frontRightMotor.setPower(-0.5);

            indexengage.setPosition(0.7);

            telemetry.update();
        }
    }
}
