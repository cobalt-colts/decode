package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

@Config
@Configurable
@TeleOp(name = "Meet 3 TeleOp")
public class meet2teleop extends LinearOpMode {
    public static boolean redAlliance = true;

    public static double intakePower;
    public static double intakeIn = 1;
    public static double intakeOut = -.5;

    public static char ball1 = 'b';
    public static char ball2 = 'b';
    public static char ball3 = 'b';
    public static char ballWant = '0';
    public enum Index {G, P, HOLD, U1, D1, U2, D2, U3, D3, DOWN}
    Index index = Index.HOLD;
    public static String indexOrder = "";
    public static double lift1Down = 0.5;
    public static double lift1Up = 0.5;
    public static double lift1UpThreshold = 0.5;
    public static double lift1DownThreshold = 0.5;
    public static double lift2Down = 0.5;
    public static double lift2Up = 0.5;
    public static double lift2UpThreshold = 0.5;
    public static double lift2DownThreshold = 0.5;
    public static double lift3Down = 0.5;
    public static double lift3Up = 0.5;
    public static double lift3UpThreshold = 0.5;
    public static double lift3DownThreshold = 0.5;
    public static double lift1Pos = lift1Down;
    public static double lift2Pos = lift2Down;
    public static double lift3Pos = lift3Down;
    public static double lift1Posi;

    public static double lift2Posi;
    public static double lift3Posi;
    public static double greenThreshold = 50;
    public static double blueThreshold = 50;

    public static double closeSpeed = 746.67;
    public static double closeHood = 0.3; // 0.25   0.35
    public static double farSpeed = 1073.33;
    public static double farHood = 0.21; // 0.1
    public static double targetTps = 0;
    public static double flywheelThreshold = 0.1;
    public static double hoodPos = 0.05; // bottom is 0.35, top is 0.

    public static double turretManual = 0.35; // 0.25
    public static int turretMax = 200;
    public static int turretMin = -80;
    public static double turretPos = 0;
    public static boolean canShoot;

    public static int limelightSlow = 250;
    public static int limelightFast = 100;

    public static double frontLeftPower;
    public static double frontRightPower;
    public static double backLeftPower;
    public static double backRightPower;

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intake;
    DcMotorEx turret, thrower1, thrower2;
    Servo hood, lift1, lift2, lift3;
    ColorSensor color11, color12, color21, color22, color31, color32;
    AnalogInput lift1Analog, lift2Analog, lift3Analog;
    Limelight3A limelight;
    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        intake = hardwareMap.dcMotor.get("intake");
        color11 = hardwareMap.get(ColorSensor.class, "color11");
        color12 = hardwareMap.get(ColorSensor.class, "color12");
        color21 = hardwareMap.get(ColorSensor.class, "color21");
        color22 = hardwareMap.get(ColorSensor.class, "color22");
        color31 = hardwareMap.get(ColorSensor.class, "color31");
        color32 = hardwareMap.get(ColorSensor.class, "color32");
        lift1 = hardwareMap.servo.get("lift1");
        lift2 = hardwareMap.servo.get("lift2");
        lift3 = hardwareMap.servo.get("lift3");
        lift1Analog = hardwareMap.get(AnalogInput.class, "lift1Analog");
        lift2Analog = hardwareMap.get(AnalogInput.class, "lift2Analog");
        lift3Analog = hardwareMap.get(AnalogInput.class, "lift3Analog");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
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
        if (gamepad1.options) {
            imu.resetYaw();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
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
        else outtake();

        if (gamepad1.yWasPressed()) ballWant = 'g';      // index = Index.G;
        if (gamepad1.xWasPressed()) ballWant = 'p';      // index = Index.P;
        if (gamepad1.bWasPressed()) ballWant = '0';      // index = Index.HOLD;
        if (gamepad1.dpadLeftWasPressed()) index = Index.U1;
        if (gamepad1.dpadUpWasPressed()) index = Index.U2;
        if (gamepad1.dpadRightWasPressed()) index = Index.U3;
        if (gamepad1.dpadDownWasPressed()) index = Index.DOWN;

        switch (index) {
//            case G:
//                if (canShoot) {
//                    if (ball1 == 'g') index = Index.U1;
//                    else if (ball2 == 'g') index = Index.U2;
//                    else if (ball3 == 'g') index = Index.U3;
//                    else index = Index.P;
//                }
//                else gamepad1.rumble(100);
//                break;
//
//            case P:
//                if (canShoot) {
//                    if (ball1 == 'p') index = Index.U1;
//                    else if (ball2 == 'p') index = Index.U2;
//                    else if (ball3 == 'p') index = Index.U3;
//                    else index = Index.G;
//                }
//                else gamepad1.rumble(100);
//                break;

            case HOLD:
                if (canShoot) {
                    if (ball1 == ballWant) index = Index.U1;
                    if (ball2 == ballWant) index = Index.U2;
                    if (ball3 == ballWant) index = Index.U3;
                }
//                lift1Pos = lift1Down;
//                lift2Pos = lift2Down;
//                lift3Pos = lift3Down;
                break;

            case U1:
                lift1Pos = lift1Up;
                if (lift1Pos == lift1Up && lift1Posi <= lift1UpThreshold) index = Index.D1;
                break;

            case D1:
                lift1Pos = lift1Down;
                if (lift1Posi >= lift1DownThreshold) index = Index.HOLD;
                break;

            case U2:
                lift2Pos = lift2Up;
                if (lift2Pos == lift2Up && lift2Posi <= lift2UpThreshold) index = Index.D2;
                break;

            case D2:
                lift2Pos = lift2Down;
                if (lift2Posi >= lift2DownThreshold) index = Index.HOLD;
                break;

            case U3:
                lift3Pos = lift3Up;
                if (lift3Pos == lift3Up && lift3Posi <= lift3UpThreshold) index = Index.D3;
                break;

            case D3:
                lift3Pos = lift3Down;
                if (lift3Posi >= lift3DownThreshold) index = Index.HOLD;
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
        lift1Posi = (lift1Analog.getVoltage() / 3.3) * 360;
        lift2Posi = (lift2Analog.getVoltage() / 3.3) * 360;
        lift3Posi = (lift3Analog.getVoltage() / 3.3) * 360;
    }
    public void intake() {intakePower = intakeIn;}
    public void outtake() {intakePower = intakeOut;}
    public void shoot() {
        canShoot = true;
        if (gamepad1.dpad_right) turretPos += turretManual;
        else if (gamepad1.dpad_left) turretPos -= turretManual;
        else if (ll.fetchAlignment(limelight, redAlliance) != 6767) turretPos += ll.fetchAlignment(limelight, redAlliance);
        else canShoot = false;
        if (turretPos > turretMax) turretPos = turretMin;
        if (turretPos < turretMin) turretPos = turretMax;
        hoodPos = (targetTps >= 1000 ? farHood : closeHood);
        if (gamepad1.right_bumper) {
            telemetry.setMsTransmissionInterval(limelightFast);
            targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
        } else if (gamepad1.dpad_up) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = closeSpeed;
            hoodPos = closeHood;
        } else if (gamepad1.dpad_down) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = farSpeed;
            hoodPos = farHood;
        } else if (gamepad1.left_trigger >= 0.2) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = 0;
        } else {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
        }
        if (thrower1.getVelocity() / targetTps >= flywheelThreshold) canShoot = false;
    }
    public void powers() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        thrower1.setVelocity(-1 * targetTps);
        thrower2.setPower(thrower1.getPower());
        turret.setTargetPosition((int) turretPos);
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