package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.*;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import static org.firstinspires.ftc.teamcode.util.ll.fetchAlignment;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.controller;
import static org.firstinspires.ftc.teamcode.util.positions.*;

import android.util.Size;

import dev.nextftc.ftc.ActiveOpMode;

@Config
//@Configurable
@TeleOp(name = "STATE Teleop")
public class STATETeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        startReady = false;
        redAlliance = false; // or whatever the default should be

        flicker1 = hardwareMap.servo.get("flicker1");
        rightAnalog = hardwareMap.analogInput.get("rightAnalog");
        flicker2 = hardwareMap.servo.get("flicker2");
        backAnalog = hardwareMap.analogInput.get("backAnalog");
        flicker3 = hardwareMap.servo.get("flicker3");
        leftAnalog = hardwareMap.analogInput.get("leftAnalog");
        flicker1Pos = flicker1down;
        flicker2Pos = flicker2down;
        flicker3Pos = flicker3down;
        flick3 = Index.HOLD;
        flick2 = Index.HOLD;
        flick1 = Index.HOLD;
        light1 = hardwareMap.servo.get("light1");
        light2 = hardwareMap.servo.get("light2");
        light3 = hardwareMap.servo.get("light3");
//        portal = new VisionPortal.Builder().addProcessor(sensor1).addProcessor(sensor2).addProcessor(sensor3).setCameraResolution(new Size(640, 480)).setCamera(hardwareMap.get(WebcamName.class, "internalcam")).build();
        posConstants.rebuildSensors();
        if (portal != null) {
            portal.close();
            portal = null;
        }
        portal = new VisionPortal.Builder()
                .addProcessor(sensor1)
                .addProcessor(sensor2)
                .addProcessor(sensor3)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "internalcam"))
                .build();

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turretPos = 0;
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(turretP);
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        controller = new PIDFController(ShootingSpeedTuning.p, ShootingSpeedTuning.i, ShootingSpeedTuning.d, ShootingSpeedTuning.f);
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thrower1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        thrower2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        targetTps = 1100;
        targetVelocity = 1100;

        hood = hardwareMap.servo.get("hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(limelightFast);

//        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        reset();

        while(!isStarted() && !isStopRequested() && !startReady) {
            if (gamepad1.optionsWasPressed()) {
                redAlliance = true;
                limelight.pipelineSwitch(2); // red pipeline
                gamepad1.setLedColor(1, 0, 0, 1000);
                startReady = true;
            }
            else if (gamepad1.shareWasPressed()) {
                redAlliance = false;
                limelight.pipelineSwitch(3); // blue pipeline
                gamepad1.setLedColor(0, 0, 1, 1000);
                startReady = true;
            }
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive();
            index();
            shoot();
            powers();
            telemetry();
            if (isStopRequested()) {
                portal.stopStreaming();
                portal.close();
            }
        }
//        portal.stopStreaming();
        portal.close();
        limelight.stop();
    }

    public void drive() {
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        if (gamepad1.share) redAlliance = false;
        if (gamepad1.optionsWasPressed()) pinpoint.resetPosAndIMU();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        botHeading += Math.PI;
        botHeading = Math.atan2(Math.sin(botHeading), Math.cos(botHeading));
        double rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

    public void index() {
        color();
        intake();

        if (gamepad1.xWasPressed()) greenPos = 1;
        if (gamepad1.yWasPressed()) greenPos = 2;
        if (gamepad1.bWasPressed()) greenPos = 3;
        if (gamepad1.aWasPressed() || gamepad1.right_trigger >= 0.2) greenPos = 4;
        if (greenPos < 1) greenPos = 3;
        if (ball1 == 'b' && ball2 == 'b' && ball3 == 'b') greenPos = 5;

        if (gamepad1.left_stick_button && gamepad1.rightStickButtonWasPressed()) {
            greenPos = 5;
            flick1 = Index.UP;
            flick2 = Index.UP;
            flick3 = Index.UP;
        }
        if (gamepad1.dpadRightWasPressed()) flick1 = Index.UP;
        if (gamepad1.dpadRightWasReleased()) flick1 = Index.DOWN;
        if (gamepad1.dpadUpWasPressed()) flick2 = Index.UP;
        if (gamepad1.dpadUpWasReleased()) flick2 = Index.DOWN;
        if (gamepad1.dpadLeftWasPressed()) flick3 = Index.UP;
        if (gamepad1.dpadLeftWasReleased()) flick3 = Index.DOWN;
        if (gamepad1.dpadDownWasPressed()) down();
        if (gamepad1.dpad_down || gamepad1.left_trigger >= 0.2 || gamepad1.aWasReleased()) greenPos = 5;


        switch (greenPos) {
            case 1:
                if (allDown) {
                    if (ball1 == 'g') {
                        flick1 = Index.AUTOUP;
                        greenPos--;
                    }
                    else if (ball2 == 'g') {
                        flick2 = Index.AUTOUP;
                        greenPos--;
                    }
                    else if (ball3 == 'g') {
                        flick3 = Index.AUTOUP;
                        greenPos--;
                    }
                    else {
                        gamepad1.rumble(200);
                    }
                }
                break;

            default:
                if (allDown) {
                    if (ball1 == 'p') {
                        flick1 = Index.AUTOUP;
                        greenPos--;
                    }
                    else if (ball2 == 'p') {
                        flick2 = Index.AUTOUP;
                        greenPos--;
                    }
                    else if (ball3 == 'p') {
                        flick3 = Index.AUTOUP;
                        greenPos--;
                    }
                    else {
                        gamepad1.rumble(200);
                    }
                }
                break;

//            case 4:
//                if (allDown) {
//                    if (ball1 != 'b') flick1 = Index.AUTOUP;
//                    else if (ball2 != 'b') flick2 = Index.AUTOUP;
//                    else if (ball3 != 'b') flick3 = Index.AUTOUP;
//                    else gamepad1.rumble(200);
//                }
//                break;
            case 4:
                if (allDown) {
                    if (ball1 != 'b') flick1 = Index.AUTOUP;
                    else if (ball2 != 'b') flick2 = Index.AUTOUP;
                    else if (ball3 != 'b') flick3 = Index.AUTOUP;
                    else {
                        gamepad1.rumble(200);
                        greenPos = 5; // all empty, stop
                    }
                    // don't decrement or reset — stay in case 4 until all slots empty
                }
                break;

            case 5:
//                down();
                break;
        }

        switch (flick2) {
            case HOLD:
                flicker2Pos = flicker2down;
                break;

            case DOWN:
                flicker2Pos = flicker2down;
                break;

            case UP:
                flicker2Pos = flicker2up;
                break;

            case TRANSFER:
                flicker2Pos = flicker2transfer;
                if (intakePower < intakeIn) flick2 = Index.AUTODOWN;
                break;

            case AUTODOWN:
                flicker2Pos = flicker2down;
                if (backAnalog.getVoltage() > flicker2DownThreshold) {
                    flick2 = Index.HOLD;
                    // removed flick3 = Index.AUTOUP
                }
                break;

            case AUTOUP:
                if (canShoot) {
                    flicker2Pos = flicker2up;
                }
                if (backAnalog.getVoltage() < flicker2UpThreshold && !gamepad1.dpad_up) flick2 = Index.AUTODOWN;
                break;
        }
        switch (flick3) {
            case HOLD:
                flicker3Pos = flicker3down;
                break;

            case DOWN:
                flicker3Pos = flicker3down;
                break;

            case UP:
                flicker3Pos = flicker3up;
                break;

            case AUTODOWN:
                flicker3Pos = flicker3down;
                if (leftAnalog.getVoltage() > flicker3DownThreshold) {
                    flick3 = Index.HOLD;
                    // removed flick1 = Index.AUTOUP
                }
                break;

            case AUTOUP:
                if (canShoot) {
                    flicker3Pos = flicker3up;
                }
                if (leftAnalog.getVoltage() < flicker3UpThreshold) flick3 = Index.AUTODOWN;
                break;
        }
        switch (flick1) {
            case HOLD:
                flicker1Pos = flicker1down;
                break;

            case DOWN:
                flicker1Pos = flicker1down;
                break;

            case UP:
                flicker1Pos = flicker1up;
                break;

            case AUTODOWN:
                flicker1Pos = flicker1down;
                if (rightAnalog.getVoltage() < flicker1DownThreshold) {
                    flick1 = Index.HOLD;
                }
                break;

            case AUTOUP:
                if (canShoot) {
                    flicker1Pos = flicker1up;
                }
                if (rightAnalog.getVoltage() > flicker1UpThreshold && !gamepad1.dpad_right) flick1 = Index.AUTODOWN;
                break;
        }
    }
    public void color() {
        //        allDown = (flicker1Pos == flicker1down && flicker3Pos == flicker3down && flicker2Pos == flicker2down && backAnalog.getVoltage() < flicker2DownThreshold && rightAnalog.getVoltage() > flicker1DownThreshold && leftAnalog.getVoltage() > flicker3DownThreshold);
        allDown = ( Math.abs(flicker1Pos - flicker1down) < 0.1
                && Math.abs(flicker3Pos - flicker3down) < 0.1
                && Math.abs(flicker2Pos - flicker2down) < 0.1
                && backAnalog.getVoltage() > flicker2DownThreshold
                && rightAnalog.getVoltage() < flicker1DownThreshold
                && leftAnalog.getVoltage() > flicker3DownThreshold);

        ball1 = getballletter(sensor1.getAnalysis(), GREY_SATURATION1);
        light1.setPosition(colorToRGBServo(sensor1.getAnalysis(), GREY_SATURATION1));

        ball2 = getballletter(sensor2.getAnalysis(), GREY_SATURATION2);
        light2.setPosition(colorToRGBServo(sensor2.getAnalysis(), GREY_SATURATION2));

        ball3 = getballletter(sensor3.getAnalysis(), GREY_SATURATION3);
        light3.setPosition(colorToRGBServo(sensor3.getAnalysis(), GREY_SATURATION3));
    }
    public void down() {
        flick3 = Index.DOWN;
        flick2 = Index.DOWN;
        flick1 = Index.DOWN;
    }
    public void intake() {
        if (gamepad1.left_bumper) intakePower = intakeOut;
        else if (gamepad1.right_bumper) {
            intakePower = intakeIn;
            flick2 = Index.TRANSFER;
        }
        else if (flicker1Pos == flicker1up || flicker2Pos == flicker2up || flicker3Pos == flicker3up) intakePower = intakeOut;
        else intakePower = ((ball1 == 'b' || ball2 == 'b' || ball3 == 'b') ? intakeIn : intakeOut);
        }
    public void shoot() {
        if (gamepad1.touchpadWasPressed()) flyWheelCorrect = 100;
        if (gamepad1.rightStickButtonWasPressed()) flyWheelCorrect -= 50;
        if (gamepad1.leftStickButtonWasPressed()) flyWheelCorrect = 0;
//        if (gamepad1.psWasPressed()) {
//            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            turretPos = 0;
//        }
        if (!magnet.getState()) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if (gamepad1.right_bumper) turretPos = turretMax;
//        else if (gamepad1.left_bumper) turretPos = turretMin;
        if (Double.isNaN(fetchAlignment(limelight, redAlliance))){
            gamepad1.rumble(0.25, 0.25, 250);
//            if (turret.getCurrentPosition() > 0) turretPos = turretMax;
//            else turretPos = turretMin;
        } else turretPos = (turret.getCurrentPosition() + fetchAlignment(limelight, redAlliance));

        turretPos = Math.min((int) turretPos, turretMax);
        turretPos = Math.max((int) turretPos, turretMin);

        controller.setPIDF(ShootingSpeedTuning.p, ShootingSpeedTuning.i, ShootingSpeedTuning.d, ShootingSpeedTuning.f);
        if (!Double.isNaN(ll.fetchFlywheelSpeed(limelight))) targetTps = (ll.fetchFlywheelSpeed(limelight)); //  * TICKS_PER_REV / 60.0
        if (targetTps <= (2000 * .88)) targetTps /= 1; //0.95
        else targetTps -= (100 - flyWheelCorrect);  // Lower top speed now that hood is shooting more horizontal. Was 150
        if (gamepad1.right_stick_button || gamepad1.left_stick_button) targetTps = -2000;
        targetVelocity = (int) targetTps;
        targetVelocity = Math.max(600, targetVelocity);
//        targetVelocity = Math.max(2000, targetVelocity);
//        targetVelocity = 800;


        currentVelocity = (thrower1.getVelocity() / 28 * 60); // /28)*60

        power = controller.calculate(currentVelocity, targetVelocity);

        thrower1.setPower(power);
        thrower2.setPower(power);

//        hoodPos = (targetTps >= 1000 ? farHood : (closeHood));
//        hoodPos = farHood;
        if (!Double.isNaN(ll.fetchHoodPos(limelight))) hoodPos = ll.fetchHoodPos(limelight);
        if (Double.isNaN(hoodPos)) hoodPos = closeHood;
//        if (count > 2) count = 0;
//        if (hoodPos > closeHood) {
//            if (count == 1) hoodPos += 0.4;
//            else if (count == 2) hoodPos += 0.3;
//        }
        hoodPos = Math.max(hoodPos, 0.08);
        hoodPos = Math.min(hoodPos, 0.4);

//        canShoot = ((thrower1.getVelocity() / targetTps <= flywheelThreshold) && (turret.getVelocity() <= turretThreshold));
        canShoot = (Math.abs((targetTps) - (thrower1.getVelocity() / 28) * 60) <= 10 && Math.abs(turretPos - turret.getCurrentPosition()) <= 5);
    }
    public void powers() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

//        thrower1.setVelocity(-1 * targetTps);
//        thrower2.setPower(thrower1.getPower());
        turret.setTargetPosition((int) turretPos);
        turret.setPower(1); //turnPower

        intake.setPower(intakePower);
        hood.setPosition(hoodPos);
//        if (gamepad1.dpad_right) {
//            flicker1.setPosition(flicker1up);
//            count++;
//        }
//        else flicker1.setPosition(flicker1down);
//        if (gamepad1.dpad_up) {
//            flicker2.setPosition(flicker2up);
//            count++;
//        }
//        else flicker2.setPosition(flicker2down);
//        if (gamepad1.dpad_left) {
//            flicker3.setPosition(flicker3up);
//            count++;
//        }
//        else flicker3.setPosition(flicker3down);
        flicker1.setPosition(flicker1Pos);
        flicker2.setPosition(flicker2Pos);
        flicker3.setPosition(flicker3Pos);
    }
    public void telemetry() {
        telemetry.addData("fetch: ", targetTps);
        telemetry.addData("currentVelocity", currentVelocity);
        telemetry.addData("diff: ", targetTps + thrower1.getVelocity());
        telemetry.addData("pid: ", power);
        telemetry.addData("ball3: ", ball3);
        telemetry.addData("ball2: ", ball2);
        telemetry.addData("ball1: ", ball1);
        telemetry.addData("greenPos: ", greenPos);
        telemetry.addData("flick2: ", backAnalog.getVoltage());
        telemetry.addData("alldown: ", allDown);
        telemetry.addData("red: ", redAlliance);
        telemetry.addData("pinpoint: ", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("thrower1velocity", thrower1.getVelocity());
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("velocitydiff", Math.abs(targetVelocity - thrower1.getVelocity()));
        telemetry.addData("thrower1power", thrower1.getPower());
        telemetry.addData("thrower2power", thrower2.getPower());
        telemetry.addData("hood: ", hoodPos);
        telemetry.addData("target turret pos: ", turretPos);
        telemetry.addData("turret:", turret.getCurrentPosition());
        telemetry.addData("Back state", (flick2.equals(Index.HOLD)));
        telemetry.addData("Left state", (flick3.equals(Index.HOLD)));
        telemetry.addData("Right state", (flick1.equals(Index.HOLD)));
//        telemetry.addData("back.getVoltage()", backAnalog.getVoltage());
//        telemetry.addData("right.getVoltage()", rightAnalog.getVoltage());
//        telemetry.addData("left.getVoltage()", leftAnalog.getVoltage());
        telemetry.update();
    }
}