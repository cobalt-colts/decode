package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
@TeleOp(name = "Meet 2.0 TeleOp")
public class derekDecodeMain extends LinearOpMode {
    public static double intakeIn = 1;
    public static double intakeOut = -0.67;
    public static double intakeTransfer = 0.5;

    private boolean isMagnet(DigitalChannel sensor1, DigitalChannel sensor2, DigitalChannel sensor3){
        if (!sensor2.getState()) { // !sensor1.getState() || !sensor2.getState() || !sensor3.getState()
            return true;
        } else {
            return false;
        }
    }
    public static boolean moveIndex = false;
    public static double manualIndex = 0.25;
    public static double autoIndex = 0.25;
    public static double indexEngaged = 0.825;
    public static double indexDisengaged = 0.7;
    public static boolean engageIndex = true;

    public static double closeSpeed = 746.67;
    public static double closeHood = 0.35; // 0.25   0.35
    public static double farSpeed = 1073.33;
    public static double farHood = 0.21; // 0.1
    double targetTps = 0;
    public static double hoodPos = 0.05; // bottom is 0.35, top is 0.


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexEngage = hardwareMap.servo.get("indexEngage");

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        DigitalChannel mag1 = hardwareMap.get(DigitalChannel.class, "mag1");
        DigitalChannel mag2 = hardwareMap.get(DigitalChannel.class, "mag2");
        DigitalChannel mag3 = hardwareMap.get(DigitalChannel.class, "mag3");



        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        Servo hood = hardwareMap.servo.get("hood");
        Servo lift = hardwareMap.servo.get("lift");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.stop();

        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
//            index(gamepad1.y, gamepad1.b, gamepad1.x, intake(gamepad1.right_bumper, gamepad1.a, moveIndex));
//            shoot(gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);


            if (gamepad1.options) {
                imu.resetYaw();
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.right_bumper) {
                intake.setPower(intakeIn);
                engageIndex = false;
            }
            else if (gamepad1.a) {
                intake.setPower(intakeOut);
                engageIndex = false;
            }
            else if (moveIndex) {
                intake.setPower(intakeTransfer);
                engageIndex = true;
            }
            else {
                intake.setPower(0);
                engageIndex = true;
            }

            if (gamepad1.y) {
                moveIndex = true;
                indexer.setPower(autoIndex);
            } else if (gamepad1.b) {
                indexer.setPower(manualIndex);
                moveIndex = false;
            }
            else if (gamepad1.x) {
                indexer.setPower(-manualIndex);
                moveIndex = false;
            }
            else if (moveIndex) {
                if (isMagnet(mag1, mag2, mag3)) {
                    indexer.setPower(0);
                    moveIndex = false;
                } else {
                    indexer.setPower(autoIndex);
//                intake(false, false, true);
                }
            } else indexer.setPower(0);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            indexEngage.setPosition(engageIndex ? indexEngaged : indexDisengaged);
            if (gamepad1.b || gamepad1.x) indexEngage.setPosition(indexEngage.getPosition() + 0.025);

            limelight.start();
            if (gamepad1.left_trigger >= 0.2) {
                targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
                double turnPower = ll.fetchAlignment(limelight);
                if (turnPower < 6767) {
                    if ((turnPower > 0 && turret.getCurrentPosition() < 200) || (turnPower < 0 && turret.getCurrentPosition() > -80)) turret.setPower(turnPower);
                } else if (turnPower == 0) gamepad1.setLedColor(0, 1, 0, 1000);
                else gamepad1.rumble(200);
            }
            if (gamepad1.right_trigger >= 0.2) {
                hoodPos = (targetTps >= 1000 ? farHood : closeHood);
            }
//            else if (gamepad1.dpad_left) {
//                targetTps = closeSpeed;
//                hoodPos = closeHood;
//            } else if (gamepad1.dpad_right) {
//                targetTps = farSpeed;
//                hoodPos = farHood;
//            }
            else {
                targetTps = 0;
            }

            if (gamepad1.dpad_left) turret.setPower(-0.25);
            else if (gamepad1.dpad_right) turret.setPower(0.25);
            else turret.setPower(0);

            if (gamepad1.left_bumper) {
                lift.setPosition(0.5);
            } else {
                lift.setPosition(0.25);
            }

            hood.setPosition(hoodPos);
            thrower1.setVelocity(targetTps);
            thrower2.setVelocity(targetTps);

            telemetry.addData("error: ", targetTps / thrower1.getVelocity());
            telemetry.addData("hood: ", hoodPos);
            telemetry.addData("target: ", targetTps);
            telemetry.addData("thrower1velocity", thrower1.getVelocity(AngleUnit.DEGREES) * 60);
            telemetry.addData("thrower2velocity", thrower2.getVelocity(AngleUnit.DEGREES) * 60);
            telemetry.addData("thrower1power", thrower1.getPower());
            telemetry.addData("thrower2power", thrower2.getPower());
            telemetry.update();
        }
    }
}