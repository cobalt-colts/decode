package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

@Config
@Configurable
@TeleOp(name = "Meet 1 TeleOp")
public class derekDecodeMain extends LinearOpMode {
    public static double intakeIn = 1;
    public static double intakeOut = -0.67;
    public static double intakeTransfer = 0.5;

    private boolean isWhite(NormalizedColorSensor sensor){
        NormalizedRGBA colors = sensor.getNormalizedColors();
        return colors.red >= 0.04 && colors.blue >= 0.04 && colors.green >= 0.04;

    }
    public static boolean moveIndex = false;
    public static double manualIndex = 0.25;
    public static double autoIndex = 0.1;
    public static double indexEngaged = 0.825;
    public static double indexDisengaged = 0.7;

    public static double closeSpeed = 746.67;
    public static double closeHood = 0.25;
    public static double farSpeed = 1073.33;
    public static double farHood = 0.15;
    double targetTps = 0;
    public static double hoodPos = 0.35; // bottom is 0.35, top is 0.


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            index(gamepad1.y, gamepad1.b, gamepad1.x, intake(gamepad1.right_bumper, gamepad1.a, moveIndex));
            shoot(gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
            telemetry.update();
        }
    }
    public void drive(double y, double x, double rx) {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        if (gamepad1.options) imu.resetYaw();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public boolean intake(boolean a, boolean b, boolean c) {
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        if (a) intake.setPower(intakeIn);
        else if (b) intake.setPower(intakeOut);
        else if (c) intake.setPower(intakeTransfer);
        else intake.setPower(0);
        return (a || b);
    }
    public void index(boolean y, boolean b, boolean x, boolean a) {
        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexEngage = hardwareMap.servo.get("indexEngage");
        NormalizedColorSensor indexSensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");
        if (y) {
            moveIndex = true;
            indexer.setPower(autoIndex);
        } else if (b) indexer.setPower(manualIndex);
        else if (x) indexer.setPower(-manualIndex);
        else if (moveIndex) {
            if (isWhite(indexSensor)) {
                indexer.setPower(0);
                moveIndex = false;
            } else {
                indexer.setPower(autoIndex);
//                intake(false, false, true);
            }
        } else indexer.setPower(0);
        indexEngage.setPosition(!a ? indexEngaged : indexDisengaged);
        NormalizedRGBA colors = indexSensor.getNormalizedColors();
        telemetry.addData("red", colors.red);
        telemetry.addData("green", colors.green);
        telemetry.addData("blue", colors.blue);
    }
    public void shoot(double trig, boolean bump, boolean up, boolean down, boolean left, boolean right) {
        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        Servo hood = hardwareMap.servo.get("hood");
        Servo lift = hardwareMap.servo.get("lift");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        if (trig >= 0.2) {
            if (bump) {
                targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
                double turnPower = ll.fetchAlignment(limelight);
                if (turnPower < 6767) drive(0, 0, turnPower);
                else gamepad1.rumble(200);
            }
            hoodPos = (targetTps >= 1000 ? .15 : .25);
        } else if (left) {
            targetTps = closeSpeed;
            hoodPos = closeHood;
        } else if (right) {
            targetTps = farSpeed;
            hoodPos = farHood;
        } else {
            targetTps = 0;
        }
        lift.setPosition(((Math.abs(targetTps / thrower1.getVelocity()) <= 1.3 || up) && !down && !bump) ? 0 : 0.9); // Teehee
        hood.setPosition(hoodPos);
        thrower1.setVelocity(targetTps);
        thrower2.setVelocity(targetTps);
        telemetry.addData("error: ", targetTps / thrower1.getVelocity());
        telemetry.addData("hood: ", hoodPos);
        telemetry.addData("target: ", targetTps);
        telemetry.addData("thrower1velocity", thrower1.getVelocity(AngleUnit.DEGREES) * 60);
        telemetry.addData("thrower2velocity", thrower2.getVelocity(AngleUnit.DEGREES) * 60);
    }
}