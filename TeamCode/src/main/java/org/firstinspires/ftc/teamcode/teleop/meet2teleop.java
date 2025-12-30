package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.util.ll;
import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.positions.*;

@Config
@Configurable
@TeleOp(name = "Meet 3 TeleOp")
public class meet2teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        intake = hardwareMap.dcMotor.get("intake");
        color11 = hardwareMap.colorSensor.get("color11");
        color12 = hardwareMap.colorSensor.get("color12");
        color21 = hardwareMap.colorSensor.get("color21");
        color22 = hardwareMap.colorSensor.get("color22");
        color31 = hardwareMap.colorSensor.get("color31");
        color32 = hardwareMap.colorSensor.get("color32");
        lift1 = hardwareMap.servo.get("lift1");
        lift2 = hardwareMap.servo.get("lift2");
        lift3 = hardwareMap.servo.get("lift3");
//        lift1Analog = hardwareMap.analogInput.get("lift1Analog");
//        lift2Analog = hardwareMap.analogInput.get("lift2Analog");
//        lift3Analog = hardwareMap.analogInput.get("lift3Analog");
//        turretMag = hardwareMap.digitalChannel.get("turretMag");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
        telemetry.setMsTransmissionInterval(limelightSlow);

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
        if (gamepad1.share) redAlliance = false;
        if (gamepad1.options) imu.resetYaw();
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        if (!autoTurret) rx = 0;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        botHeading = Math.toRadians(180);
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

        if (gamepad1.right_bumper) intake();
        else if (gamepad1.a) outtake();
        else autoIntake();

        if (gamepad1.yWasPressed()) ballWant = 'g';
        if (gamepad1.xWasPressed()) ballWant = 'p';
        if (gamepad1.bWasPressed()) ballWant = '0';
        if (gamepad2.yWasPressed()) indexOrder += "g";
        if (gamepad2.xWasPressed()) indexOrder += "p";
        if (gamepad2.bWasPressed()) indexOrder = "";
        if (!indexOrder.isEmpty()) ballWant = indexOrder.charAt(0);
        if (gamepad1.dpadLeftWasPressed()) index = Index.U1;
        if (gamepad1.dpadUpWasPressed()) index = Index.U2;
        if (gamepad1.dpadRightWasPressed()) index = Index.U3;
        if (gamepad1.dpad_down) index = Index.DOWN;
        liftTime--;

        switch (index) {

            case HOLD:
                if (canShoot) {
                    if (ball1 == ballWant) {
                        liftTime = liftUpTime;
                        index = Index.U1;
                    }
                    else if (ball2 == ballWant) {
                        liftTime = liftUpTime;
                        index = Index.U2;
                    }
                    else if (ball3 == ballWant) {
                        liftTime = liftUpTime;
                        index = Index.U3;
                    }
                    else ballWant = (ballWant == 'g' ? 'p' : 'g');
                } else index = Index.DOWN;
                break;

            case U1:
                lift1Pos = lift1Up;
//                if (lift1Pos == lift1Up && lift1Posi <= lift1UpThreshold) index = Index.D1;
                if (lift1Pos == lift1Up && liftTime <= 0) {
                    if (!indexOrder.isEmpty()) indexOrder = indexOrder.substring(1);
                    liftTime = liftDownTime;
                    index = Index.D1;
                }
                break;

            case D1:
                lift1Pos = lift1Down;
//                if (lift1Posi >= lift1DownThreshold) index = Index.HOLD;
                if (liftTime <= 0) index = Index.HOLD;
                break;

            case U2:
                lift2Pos = lift2Up;
//                if (lift2Pos == lift2Up && lift2Posi <= lift2UpThreshold) index = Index.D2;
                if (lift2Pos == lift2Up && liftTime <= 0) {
                    if (!indexOrder.isEmpty()) indexOrder = indexOrder.substring(1);
                    liftTime = liftDownTime;
                    index = Index.D2;
                }
                break;

            case D2:
                lift2Pos = lift2Down;
//                if (lift2Posi >= lift2DownThreshold) index = Index.HOLD;
                if (liftTime <= 0) index = Index.HOLD;
                break;

            case U3:
                lift3Pos = lift3Up;
//                if (lift3Pos == lift3Up && lift3Posi <= lift3UpThreshold) index = Index.D3;
                if (lift3Pos == lift3Up && liftTime <= 0) {
                    if (!indexOrder.isEmpty()) indexOrder = indexOrder.substring(1);
                    liftTime = liftDownTime;
                    index = Index.D3;
                }
                break;

            case D3:
                lift3Pos = lift3Down;
//                if (lift3Posi >= lift3DownThreshold) index = Index.HOLD;
                if (liftTime <= 0) index = Index.HOLD;
                break;

            case DOWN:
                lift1Pos = lift1Down;
                lift2Pos = lift2Down;
                lift3Pos = lift3Down;
        }
    }
    public void color() {
        if (color11.green() >= greenThreshold || color12.green() >= greenThreshold) ball1 = 'g';
        else if (color11.blue() >= blueThreshold || color12.blue() >= blueThreshold) ball1 = 'p';
        else ball1 = 'b';
        if (color21.green() >= greenThreshold || color22.green() >= greenThreshold) ball2 = 'g';
        else if (color21.blue() >= blueThreshold || color22.blue() >= blueThreshold) ball2 = 'p';
        else ball2 = 'b';
        if (color31.green() >= greenThreshold || color32.green() >= greenThreshold) ball3 = 'g';
        else if (color31.blue() >= blueThreshold || color32.blue() >= blueThreshold) ball3 = 'p';
        else ball3 = 'b';
//        lift1Posi = (lift1Analog.getVoltage() / 3.3) * 360;
//        lift2Posi = (lift2Analog.getVoltage() / 3.3) * 360;
//        lift3Posi = (lift3Analog.getVoltage() / 3.3) * 360;
    }
    public void intake() {intakePower = intakeIn;}
    public void autoIntake() {intakePower = ((ball1 == 'b' || ball2 == 'b' || ball3 == 'b') ? intakeIn : intakeOut);}
    public void outtake() {intakePower = intakeOut;}
    public void shoot() {
        aim();
        speed();
        canShoot = ((thrower1.getVelocity() / targetTps <= flywheelThreshold) && (turret.getVelocity() <= turretThreshold));
    }
    public void aim() {
//        if (!turretMag.getState()) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (gamepad1.psWasPressed()) {
            if (!autoTurret) turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            autoTurret = !autoTurret;
        }
        if (gamepad1.dpad_right) turretPos += turretManual;
        else if (gamepad1.dpad_left) turretPos -= turretManual;
        else if (ll.fetchAlignment(limelight, redAlliance) != 6767) turretPos += ll.fetchAlignment(limelight, redAlliance);
        if (turretPos > turretMax) turretPos = turretMin;
        if (turretPos < turretMin) turretPos = turretMax;

        hoodPos = (targetTps >= 1000 ? farHood : closeHood);
    }
    public void speed() {
        if (gamepad1.right_trigger >= 0.2) {
            telemetry.setMsTransmissionInterval(limelightFast);
            targetTps = ll.fetchFlywheelSpeed(limelight) * TICKS_PER_REV / 60.0;
        } else if (gamepad1.left_trigger >= 0.2) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = 0;
        } else {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = ll.fetchFlywheelSpeed(limelight) * TICKS_PER_REV / 60.0;
        }
    }
    public void powers() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        thrower1.setVelocity(-1 * targetTps);
        thrower2.setPower(thrower1.getPower());
        if (autoTurret) {
            turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turret.setTargetPosition((int) turretPos);
        }
        else {
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret.setPower(turretManual * gamepad1.right_stick_x);
        }
        intake.setPower(intakePower);
        hood.setPosition(hoodPos);
        lift1.setPosition(lift1Pos);
        lift2.setPosition(lift2Pos);
        lift3.setPosition(lift3Pos);
    }
    public void telemetry() {
        telemetry.addData("thrower1velocity", thrower1.getVelocity(AngleUnit.DEGREES) * 60);
        telemetry.addData("thrower2velocity", thrower2.getVelocity(AngleUnit.DEGREES) * 60);
        telemetry.addData("thrower1power", thrower1.getPower());
        telemetry.addData("thrower2power", thrower2.getPower());
        telemetry.addData("hood: ", hoodPos);
        telemetry.addData("target: ", targetTps);
        telemetry.update();
    }
}