package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.*;
import org.firstinspires.ftc.teamcode.util.*;

import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.*;
import static org.firstinspires.ftc.teamcode.util.ll.fetchAlignment;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.positions.*;
import static org.firstinspires.ftc.teamcode.util.teleSubsystems.*;

@Config
@Configurable
@TeleOp(name = "State Teleop")
public class StateTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

//        right1 = hardwareMap.colorSensor.get("right1");
//        right2 = hardwareMap.colorSensor.get("right2");
//        back1 = hardwareMap.colorSensor.get("back1");
//        back2 = hardwareMap.colorSensor.get("back2");
//        left1 = hardwareMap.colorSensor.get("left1");
//        left2 = hardwareMap.colorSensor.get("left2");

        rightFlicker = hardwareMap.servo.get("flicker1");
        rightAnalog = hardwareMap.analogInput.get("rightAnalog");
        backFlicker = hardwareMap.servo.get("flicker2");
        backAnalog = hardwareMap.analogInput.get("backAnalog");
        leftFlicker = hardwareMap.servo.get("flicker3");
//        leftAnalog = hardwareMap.analogInput.get("leftAnalog");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(turretP);

        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        thrower2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

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
                gamepad1.setLedColor(1, 0, 0, 1000);
                startReady = true;
            }
            else if (gamepad1.shareWasPressed()) {
                redAlliance = false;
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

        if (gamepad1.aWasPressed()) ballNum = 0;

        if (gamepad1.dpad_down) down();

//        allDown = ((rightFlickerPos == rightFlickerDown && rightAnalog.getVoltage() > rightFlickerDownThreshold) && (backFlickerPos == backFlickerDown && backAnalog.getVoltage() < backFlickerDownThreshold) && (leftFlickerPos == leftFlickerDown && leftAnalog.getVoltage() < leftFlickerDownThreshold));
//        allDown = true;
        allDown = (leftFlickerPos == leftFlickerDown && rightFlickerPos == rightFlickerDown && backFlickerPos == backFlickerDown && backAnalog.getVoltage() < backFlickerDownThreshold && rightAnalog.getVoltage() > rightFlickerDownThreshold);
        switch (back) {
            case HOLD:
                if (allDown) {
                    if ((gamepad1.dpadUpWasPressed()) || ((backBall == ballWant[0] || backBall == ballWant[1]) && canShoot)) {
                        ballWant[0] = ' ';
                        ballWant[1] = ' ';
                        back = Index.UP;
                        ballNum++;
                    }
                }
                break;

            case DOWN:
                backFlickerPos = backFlickerDown;
//                back = Index.HOLD;
                if (backAnalog.getVoltage() < backFlickerDownThreshold) back = Index.HOLD;
                break;

            case UP:
                backFlickerPos = backFlickerUp;
//                if (gamepad1.dpadUpWasReleased()) back = Index.DOWN;
                if (backAnalog.getVoltage() > backFlickerUpThreshold && !gamepad1.dpad_up) back = Index.DOWN;
                balls[1] = 'b';
                break;
        }
        switch (left) {
            case HOLD:
                if (allDown) {
                    if ((gamepad1.dpadLeftWasPressed()) || ((leftBall == ballWant[0] || leftBall == ballWant[1]) && canShoot)) {
                        ballWant[0] = ' ';
                        ballWant[1] = ' ';
                        left = Index.UP;
                        ballNum++;
                    }
                }
                break;

            case DOWN:
                leftFlickerPos = leftFlickerDown;
                left = Index.HOLD;
//                if (leftAnalog.getVoltage() < leftFlickerDownThreshold) left = Index.HOLD;
                break;

            case UP:
                leftFlickerPos = leftFlickerUp;
                if (gamepad1.dpadLeftWasReleased()) left = Index.DOWN;
                balls[0] = 'b';
//                if (leftAnalog.getVoltage() > leftFlickerUpThreshold) left = Index.DOWN;
                break;
        }
        switch (right) {
            case HOLD:
                if (allDown) {
                    if ((gamepad1.dpadRightWasPressed()) || ((rightBall == ballWant[0] || rightBall == ballWant[1]) && canShoot)) {
                        ballWant[0] = ' ';
                        ballWant[1] = ' ';
                        right = Index.UP;
                        ballNum++;
                    }
                }
                break;

            case DOWN:
                rightFlickerPos = rightFlickerDown;
//                right = Index.HOLD;
                if (rightAnalog.getVoltage() > rightFlickerDownThreshold) right = Index.HOLD;
                break;

            case UP:
                rightFlickerPos = rightFlickerUp;
                if (rightAnalog.getVoltage() < rightFlickerUpThreshold && !gamepad1.dpad_right) right = Index.DOWN;
                balls[2] = 'b';
                break;
        }
    }

    public void down() {
        left = Index.DOWN;
        back = Index.DOWN;
        right = Index.DOWN;
    }
    //    public void color() {
//        if (leftBall == 'B') leftBall = getColor(left1, left1, 0);
//        if (backBall == 'B') backBall = getColor(back1, back2, 1);
//        if (rightBall == 'B') rightBall = getColor(right1, right1, 2);
//        corralOrder = "" + leftBall + backBall + rightBall;
//        switch (corralOrder) {
//            case ("PPP"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//                break;
//
//            case ("PPG"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
//                break;
//
//            case ("PGG"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
//                break;
//
//            case ("PGP"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
//                break;
//
//            case ("GPP"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
//                break;
//
//            case ("GGP"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);
//                break;
//
//            case ("GPG"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_6);
//                break;
//
//            case ("GGG"):
//                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);
//                break;
//
//        }
//    }
    public void intake() {
        if (gamepad1.a) intakePower = intakeOut;
        else intakePower = ((rightBall == 'b' || backBall == 'b' || leftBall == 'b') ? intakeIn : intakeOut);
        if (rightFlickerPos == rightFlickerUp || backFlickerPos == backFlickerUp || leftFlickerPos == leftFlickerUp) intakePower = intakeOut;
    }
    public void shoot() {
        if (gamepad1.touchpadWasPressed()) flyWheelCorrect = 100;
        if (gamepad1.leftStickButtonWasPressed()) flyWheelCorrect = 0;
        if (gamepad1.psWasPressed()) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretPos = 0;
        }
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad1.right_bumper) turretPos = turretMax;
        else if (gamepad1.left_bumper) turretPos = turretMin;
        else if (Double.isNaN(fetchAlignment(limelight, redAlliance))){
            gamepad1.rumble(0.25, 0.25, 250);
//            if (turret.getCurrentPosition() > 0) turretPos = turretMax;
//            else turretPos = turretMin;
        } else turretPos = (turret.getCurrentPosition() + fetchAlignment(limelight, redAlliance));

        turretPos = Math.min((int) turretPos, turretMax);
        turretPos = Math.max((int) turretPos, turretMin);

        targetTps = ll.fetchFlywheelSpeed(limelight) * TICKS_PER_REV / 60.0;

        hoodPos = (targetTps >= 1000 ? farHood : (closeHood));
        if (hoodPos == closeHood) hoodAdjust = hoodAdjustClose;
        else hoodAdjust = 0;
        hoodPos += Math.min(2, ballNum) * hoodAdjust;

//        canShoot = ((thrower1.getVelocity() / targetTps <= flywheelThreshold) && (turret.getVelocity() <= turretThreshold));
        if (hoodPos == farHood) canShoot = thrower1.getVelocity() <= -800;
        else canShoot = true;
    }
    public void powers() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        thrower1.setVelocity(-1 * targetTps);
        thrower2.setPower(thrower1.getPower());
        turret.setTargetPosition((int) turretPos);
        turret.setPower(1); //turnPower

        intake.setPower(intakePower);
        hood.setPosition(hoodPos);
        rightFlicker.setPosition(rightFlickerPos);
        backFlicker.setPosition(backFlickerPos);
        leftFlicker.setPosition(leftFlickerPos);
    }
    public void telemetry() {
        telemetry.addData("fetch: ", targetTps);
        telemetry.addData("thrower21elocity", thrower1.getVelocity());
        telemetry.addData("diff: ", targetTps + thrower1.getVelocity());
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
        telemetry.update();
    }
}