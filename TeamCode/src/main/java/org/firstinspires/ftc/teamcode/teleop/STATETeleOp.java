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
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.*;
import org.firstinspires.ftc.teamcode.util.*;

import static org.firstinspires.ftc.teamcode.util.ll.fetchAlignment;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.controller;
import static org.firstinspires.ftc.teamcode.util.positions.*;

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

        rightFlicker = hardwareMap.servo.get("flicker1");
        rightAnalog = hardwareMap.analogInput.get("rightAnalog");
        backFlicker = hardwareMap.servo.get("flicker2");
        backAnalog = hardwareMap.analogInput.get("backAnalog");
        leftFlicker = hardwareMap.servo.get("flicker3");
        leftAnalog = hardwareMap.analogInput.get("leftAnalog");
        rightFlickerPos = flicker1down;
        backFlickerPos = flicker2down;
        leftFlickerPos = flicker3down;
        left = Index.HOLD;
        back = Index.HOLD;
        right = Index.HOLD;

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
//        color();
        intake();

        if (gamepad1.yWasPressed()) ballWant[0] = 'g';
        if (gamepad1.xWasPressed()) ballWant[0] = 'p';
//        if (gamepad1.aWasPressed()) ballWant[0] = ' ';
        if (gamepad1.b) {
            ballWant[0] = 'g';
            ballWant[1] = 'p';
        }
        else ballWant[1] = ballWant[0];

        if (gamepad1.dpadUpWasPressed()) back = Index.UP;

        if (gamepad1.aWasPressed()) ballNum = 0;

        if (gamepad1.dpad_down) down();

//        rightFlickerPos = (gamepad1.dpad_right ? flicker1up : flicker1down);
//        backFlickerPos = (gamepad1.dpad_up ? flicker2up : flicker2down);
//        leftFlickerPos = (gamepad1.dpad_left ? flicker3up : flicker3down);

//        allDown = ((rightFlickerPos == rightFlickerDown && rightAnalog.getVoltage() > rightFlickerDownThreshold) && (backFlickerPos == backFlickerDown && backAnalog.getVoltage() < backFlickerDownThreshold) && (leftFlickerPos == leftFlickerDown && leftAnalog.getVoltage() < leftFlickerDownThreshold));
//        allDown = true;
        allDown = (leftFlickerPos == flicker1down && rightFlickerPos == flicker3down && backFlickerPos == flicker2down && backAnalog.getVoltage() < backFlickerDownThreshold && rightAnalog.getVoltage() > rightFlickerDownThreshold);
        switch (back) {
            case HOLD:
//                if (allDown) {
//                    if (gamepad1.dpadUpWasPressed()) {
//                        ballWant[0] = ' ';
//                        ballWant[1] = ' ';
//                        back = Index.UP;
//                        ballNum++;
//                    }
//                }
//                left = Index.UP;
                backFlickerPos = flicker2down;
                break;

            case DOWN:
                backFlickerPos = flicker2down;
//                back = Index.HOLD;
                if (backAnalog.getVoltage() > backFlickerDownThreshold) {
                    back = Index.HOLD;
                    left = Index.UP;
                }
                break;

            case UP:
                backFlickerPos = flicker2up;
//                if (gamepad1.dpadUpWasReleased()) back = Index.DOWN;
                if (backAnalog.getVoltage() < backFlickerUpThreshold && !gamepad1.dpad_up) back = Index.DOWN;
//                balls[1] = 'b';
                break;
        }
        switch (left) {
            case HOLD:
//                if (allDown) {
//                    if (gamepad1.dpadLeftWasPressed()) left = Index.UP;
//                }
//                right = Index.UP;
                leftFlickerPos = flicker3down;
                break;

            case DOWN:
                leftFlickerPos = flicker3down;
//                left = Index.HOLD;
                if (leftAnalog.getVoltage() > leftFlickerDownThreshold) {
                    left = Index.HOLD;
                    right = Index.UP;
                }
                break;

            case UP:
                leftFlickerPos = flicker3up;
//                if (gamepad1.dpadLeftWasReleased()) left = Index.DOWN;
                if (leftAnalog.getVoltage() < leftFlickerUpThreshold) left = Index.DOWN;
                break;
        }
        switch (right) {
            case HOLD:
//                if (allDown) {
//                    if (gamepad1.dpadRightWasPressed()) {
//                        ballWant[0] = ' ';
//                        ballWant[1] = ' ';
//                        right = Index.UP;
//                        ballNum++;
//                    }
//                }
                rightFlickerPos = flicker1down;
                break;

            case DOWN:
                rightFlickerPos = flicker1down;
//                right = Index.HOLD;
                if (rightAnalog.getVoltage() < rightFlickerDownThreshold) {
                    right = Index.HOLD;
                }
                break;

            case UP:
                rightFlickerPos = flicker1up;
                if (rightAnalog.getVoltage() > rightFlickerUpThreshold && !gamepad1.dpad_right) right = Index.DOWN;
//                balls[2] = 'b';
                break;
        }
    }
    public void down() {
        left = Index.HOLD;
        back = Index.HOLD;
        right = Index.HOLD;
    }
    public void intake() {
        if (gamepad1.left_bumper) intakePower = intakeOut;
        else if (gamepad1.right_bumper) intakePower = intakeIn;
        else intakePower = ((rightBall == 'b' || backBall == 'b' || leftBall == 'b') ? intakeIn : intakeOut);
        if (rightFlickerPos == flicker3up || backFlickerPos == flicker2up || leftFlickerPos == flicker1up) intakePower = intakeOut;
    }
    public void shoot() {
        if (gamepad1.touchpadWasPressed()) flyWheelCorrect = 100;
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
        if (targetTps != 1691) targetTps /= 0.95;
        else targetTps -= 50;
        if (gamepad1.right_stick_button) targetTps = -2000;
        targetVelocity = (int) targetTps;
//        targetVelocity = 800;


        currentVelocity = (thrower1.getVelocity() / 28 * 60); // /28)*60

        power = controller.calculate(currentVelocity, targetVelocity);

        if (targetVelocity < 100) {
            thrower1.setMotorDisable();
            thrower2.setMotorDisable();
        }
        else {
            thrower1.setPower(power);
            thrower2.setPower(power);
        }

//        hoodPos = (targetTps >= 1000 ? farHood : (closeHood));
//        hoodPos = farHood;
        if (!Double.isNaN(ll.fetchHoodPos(limelight))) hoodPos = ll.fetchHoodPos(limelight);
        if (Double.isNaN(hoodPos)) hoodPos = closeHood;
//        if (count > 2) count = 0;
//        if (hoodPos > closeHood) {
//            if (count == 1) hoodPos += 0.4;
//            else if (count == 2) hoodPos += 0.3;
//        }
        hoodPos = Math.max(hoodPos, 0.19);
        hoodPos = Math.min(hoodPos, 0.4);

//        canShoot = ((thrower1.getVelocity() / targetTps <= flywheelThreshold) && (turret.getVelocity() <= turretThreshold));
        if (hoodPos == farHood) canShoot = (targetTps) - (thrower1.getVelocity() / 28) * 60 <= 10;
        else canShoot = true;
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
//            rightFlicker.setPosition(flicker1up);
//            count++;
//        }
//        else rightFlicker.setPosition(flicker1down);
//        if (gamepad1.dpad_up) {
//            backFlicker.setPosition(flicker2up);
//            count++;
//        }
//        else backFlicker.setPosition(flicker2down);
//        if (gamepad1.dpad_left) {
//            leftFlicker.setPosition(flicker3up);
//            count++;
//        }
//        else leftFlicker.setPosition(flicker3down);
        rightFlicker.setPosition(rightFlickerPos);
        backFlicker.setPosition(backFlickerPos);
        leftFlicker.setPosition(leftFlickerPos);
    }
    public void telemetry() {
        telemetry.addData("fetch: ", targetTps);
        telemetry.addData("thrower1Velocity", currentVelocity);
        telemetry.addData("diff: ", targetTps + thrower1.getVelocity());
        telemetry.addData("pid: ", power);
        telemetry.addData("left: ", leftBall);
        telemetry.addData("back: ", backBall);
        telemetry.addData("right: ", rightBall);
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