package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.*;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import static org.firstinspires.ftc.teamcode.util.ll.fetchAlignment;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.controller;
import static org.firstinspires.ftc.teamcode.util.positions.*;

@Config
@Configurable
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

        flicker1 = hardwareMap.servo.get("flicker1");
        rightAnalog = hardwareMap.analogInput.get("rightAnalog");
        flicker2 = hardwareMap.servo.get("flicker2");
        backAnalog = hardwareMap.analogInput.get("backAnalog");
        flicker3 = hardwareMap.servo.get("flicker3");
        leftAnalog = hardwareMap.analogInput.get("leftAnalog");
        flicker1Pos = flicker1down;
        flicker2Pos = flicker2down;
        flicker3Pos = flicker3down;

        light1 = hardwareMap.servo.get("light1");
        light2 = hardwareMap.servo.get("light2");
        light3 = hardwareMap.servo.get("light3");

        magnet = hardwareMap.digitalChannel.get("magnet");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(turretP);

        controller = new PIDFController(ShootingSpeedTuning.p, ShootingSpeedTuning.i, ShootingSpeedTuning.d, ShootingSpeedTuning.f);
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thrower1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        thrower2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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
        }

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

        if (gamepad1.dpadUpWasPressed()) back = Index.UP;
        if (gamepad1.aWasPressed()) index = Index.ANY;
        if (gamepad1.xWasPressed()) {
//            greenPos = 1;
            index = Index.GREEN;
        }
        if (gamepad1.yWasPressed()) {
//            greenPos = 2;
        }
        if (gamepad1.bWasPressed()) {
//            greenPos = 3;
            index = Index.PURPLE;
        }
        if (gamepad1.dpad_down) {
            index = Index.HOLD;
            down();
        }

//        switch (greenPos) {
//            case 1:
//                if (allDown) {
//                    index = Index.GREEN;
//                    greenPos = 3;
//                }
//                break;
//
//            default:
//                if (allDown) {
//                    index = Index.PURPLE;
//                    greenPos--;
//                }
//                break;
//        }

        switch (index) {
            case GREEN:
                if (allDown) {
                    if (color[0] == 'g') {
                        back = Index.UP;
                        index = Index.HOLD;
                    }
                    else if (color[1] == 'g') {
                        left = Index.UP;
                        index = Index.HOLD;
                    }
                    else if (color[2] == 'g') {
                        right = Index.UP;
                        index = Index.HOLD;
                    }
                    else {
                        if (!isOccupied[0] && !isOccupied[1] && !isOccupied[2]) {
                            gamepad1.rumble(200);
                            index = Index.HOLD;
                        }
                    }
                }
                else {
                    if (!isOccupied[0] && !isOccupied[1] && !isOccupied[2]) {
                        gamepad1.rumble(200);
                        index = Index.HOLD;
                    }
                }
                break;

            case PURPLE:
                if (allDown) {
                    if (color[0] == 'p') {
                        back = Index.UP;
                        index = Index.HOLD;
                    }
                    else if (color[1] == 'p') {
                        left = Index.UP;
                        index = Index.HOLD;
                    }
                    else if (color[2] == 'p') {
                        right = Index.UP;
                        index = Index.HOLD;
                    }
                    else {
                        if (!isOccupied[0] && !isOccupied[1] && !isOccupied[2]) {
                            gamepad1.rumble(200);
                            index = Index.HOLD;
                        }
                    }
                }
                else {
                    if (!isOccupied[0] && !isOccupied[1] && !isOccupied[2]) {
                        gamepad1.rumble(200);
                        index = Index.HOLD;
                    }
                }
                break;

            case ANY:
                if (allDown) {
                    if (isOccupied[0]) back = Index.UP;
                    else if (isOccupied[1]) left = Index.UP;
                    else if (isOccupied[2]) right = Index.UP;
                    else {
                        gamepad1.rumble(200);
                        index = Index.HOLD;
                    }
                }
                else {
                    if (!isOccupied[0] && !isOccupied[1] && !isOccupied[2]) {
                        gamepad1.rumble(200);
                        index = Index.HOLD;
                    }
                }
                break;

            case HOLD:
                break;
        }

        switch (back) {
            case HOLD:
                break;

            case DOWN:
                flicker2Pos = flicker2down;
                if (backAnalog.getVoltage() > flicker2DownThreshold) {
                    back = Index.HOLD;
                    left = Index.UP;
                }
                break;

            case UP:
                if (canShoot) {
                    flicker2Pos = flicker2up;
                    if (backAnalog.getVoltage() < flicker2UpThreshold && !gamepad1.dpad_up) back = Index.DOWN;
                }
                break;
        }
        switch (left) {
            case HOLD:
                break;

            case DOWN:
                flicker3Pos = flicker3down;
                if (leftAnalog.getVoltage() > flicker3DownThreshold) {
                    left = Index.HOLD;
                    right = Index.UP;
                }
                break;

            case UP:
                if (canShoot) {
                    flicker3Pos = flicker3up;
                    if (leftAnalog.getVoltage() < flicker3UpThreshold) left = Index.DOWN;
                }
                break;
        }
        switch (right) {
            case HOLD:
                break;

            case DOWN:
                flicker1Pos = flicker1down;
                if (rightAnalog.getVoltage() < flicker1DownThreshold) {
                    right = Index.HOLD;
                }
                break;

            case UP:
                if (canShoot) {
                    flicker1Pos = flicker1up;
                    if (rightAnalog.getVoltage() > flicker1UpThreshold && !gamepad1.dpad_right) right = Index.DOWN;
                }
                break;
        }
    }
    public void down() {
        left = Index.DOWN;
        back = Index.DOWN;
        right = Index.DOWN;
    }
    public void color() {
        result1 = sensor1.getAnalysis();
        result2 = sensor2.getAnalysis();
        result3 = sensor3.getAnalysis();

        color[0] = cameraColor(result1);
        isOccupied[0] = color[0] != 'b';
        light(light1, color[0]);
        color[1] = cameraColor(result2);
        isOccupied[1] = color[1] != 'b';
        light(light2, color[1]);
        color[2] = cameraColor(result3);
        isOccupied[2] = color[2] != 'b';
        light(light3, color[2]);
    }
    public static void light(Servo indicator, char color) {
        switch (color) {
            case 'g':
                indicator.setPosition(canShoot ? green : yellow);
                break;
            case 'p':
                indicator.setPosition(canShoot ? purple : blue);
                break;
            case 'b':
                indicator.setPosition(red);
                break;
        }
    }
    private static char cameraColor(PredominantColorProcessor.Result result) {
        if (result == null) return 'b';
        PredominantColorProcessor.Swatch resultswatch = result.closestSwatch;
        if (resultswatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) return 'g';
        else if (resultswatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) return 'p';
        else return 'b';
    }
    public void intake() {
        if (gamepad1.left_bumper) intakePower = intakeOut;
        else if (gamepad1.right_bumper) intakePower = intakeIn;
        else intakePower = ((!isOccupied[0] || !isOccupied[1] || !isOccupied[2]) ? intakeIn : intakeOut);
        if (flicker1Pos == flicker3up || flicker2Pos == flicker2up || flicker3Pos == flicker1up) intakePower = intakeOut;
    }
    public void shoot() {
        if (gamepad1.touchpadWasPressed()) flyWheelCorrect = 100;
        if (gamepad1.leftStickButtonWasPressed()) flyWheelCorrect = 0;
        if (gamepad1.psWasPressed()) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretPos = 0;
        }
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fetchTurret = fetchAlignment(limelight, redAlliance);

        if (!Double.isNaN(fetchTurret)) turretPos = (turret.getCurrentPosition() + fetchTurret);
        else if (gamepad1.right_bumper) turretPos += turretManual;
        else if (gamepad1.left_bumper) turretPos -= turretManual;
        else gamepad1.rumble(0.25, 0.25, 250);

        turretPos = Math.min((int) turretPos, turretMax);
        turretPos = Math.max((int) turretPos, turretMin);

        controller.setPIDF(ShootingSpeedTuning.p, ShootingSpeedTuning.i, ShootingSpeedTuning.d, ShootingSpeedTuning.f);
        if (!Double.isNaN(ll.fetchFlywheelSpeed(limelight))) targetTps = (ll.fetchFlywheelSpeed(limelight));
        targetTps += flyWheelCorrect;
        targetVelocity = (int) targetTps;

        currentVelocity = (thrower1.getVelocity() / 28 * 60);

        power = controller.calculate(currentVelocity, targetVelocity);

        if (targetVelocity < 100) {
            thrower1.setMotorDisable();
            thrower2.setMotorDisable();
        }
        else {
            thrower1.setPower(power);
            thrower2.setPower(power);
        }

        if (!Double.isNaN(ll.fetchHoodPos(limelight))) hoodPos = ll.fetchHoodPos(limelight);
        if (Double.isNaN(hoodPos)) hoodPos = closeHood;
        hoodPos = Math.max(hoodPos, 0.19);
        hoodPos = Math.min(hoodPos, 0.4);

        canShoot = Math.abs(targetVelocity - currentVelocity) <= 10;
    }
    public void powers() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        turret.setTargetPosition((int) turretPos);
        turret.setPower(1);

        intake.setPower(intakePower);
        hood.setPosition(hoodPos);
        flicker1.setPosition(flicker1Pos);
        flicker2.setPosition(flicker2Pos);
        flicker3.setPosition(flicker3Pos);
    }
    public void telemetry() {
        telemetry.addData("fetch: ", targetTps);
        telemetry.addData("thrower1Velocity", currentVelocity);
        telemetry.addData("diff: ", targetTps + thrower1.getVelocity());
        telemetry.addData("pid: ", power);
        telemetry.addData("left: ", cameraColor(result3));
        telemetry.addData("back: ", cameraColor(result2));
        telemetry.addData("right: ", cameraColor(result1));
        telemetry.addData("back: ", backAnalog.getVoltage());
        telemetry.addData("red: ", redAlliance);
        telemetry.addData("pinpoint: ", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("thrower1velocity", thrower1.getVelocity());
        telemetry.addData("thrower1power", thrower1.getPower());
        telemetry.addData("thrower2power", thrower2.getPower());
        telemetry.addData("hood: ", hoodPos);
        telemetry.addData("target turret pos: ", turretPos);
        telemetry.addData("turret:", turret.getCurrentPosition());
        telemetry.addData("Back state", (back.equals(Index.HOLD)));
        telemetry.addData("Left state", (left.equals(Index.HOLD)));
        telemetry.addData("Right state", (right.equals(Index.HOLD)));
        telemetry.update();
    }
}