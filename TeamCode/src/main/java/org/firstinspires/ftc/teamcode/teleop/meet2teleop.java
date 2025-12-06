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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

@Config
@Configurable
@TeleOp(name = "Meet 2.0 TeleOp")
public class meet2teleop extends LinearOpMode {
    public static boolean redAlliance = true;

    public static double intakePower;
    public static double intakeIn = 1;
    public static double intakeOut = -0.67;
    public static double intakeTransfer = 0.5;

//    private boolean isMagnet(DigitalChannel sensor1, DigitalChannel sensor2, DigitalChannel sensor3){
//        if (!sensor2.getState()) { // !sensor1.getState() || !sensor2.getState() || !sensor3.getState()
//            return true;
//        } else {
//            return false;
//        }
//    }
    public enum Index {
        AUTO,
        FORWARD,
        BACKWARD,
        SHOOT,
        HOLD,
        DISENGAGE,
        INTAKE
    }
    Index indexState = Index.HOLD;
    public static int indexTimer = 0;
    public static boolean moveIndex = false;
    public static double manualIndex = 0.25;
    public static double autoIndex = -0.1;  // -0.25     // Normal index speed
    public static double correctIndex = 0.1;    //  0.1    Fine adjust speed
    public static double indexEngaged = 0.84; //0.825
    public static double indexDisengaged = 0.7;
    public static double indexEngagePos = indexEngaged;
    public static double indexPower;
    public static boolean indexReady = false;
    public static boolean engageIndex = true;

    public static double liftUp = 0.5;
    public static double liftDown = 0.25;
    public  static double liftPos = liftDown;

    public static double closeSpeed = 746.67;
    public static double closeHood = 0.3; // 0.25   0.35
    public static double farSpeed = 1073.33;
    public static double farHood = 0.21; // 0.1
    double targetTps = 0;
    public static double hoodPos = 0.05; // bottom is 0.35, top is 0.

    public static double turretManual = 0.35; // 0.25
    public static double turretPower = 0;

    public static int limelightSlow = 250;
    public static int limelightFast = 100;

    public static double frontLeftPower;
    public static double frontRightPower;
    public static double backLeftPower;
    public static double backRightPower;

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intake;
    DcMotorEx turret, thrower1, thrower2;
    Servo indexEngage, hood, lift;
    CRServo indexer;
    DigitalChannel magnet1, magnet2, magnet3;
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

        indexer = hardwareMap.crservo.get("indexer");
        indexEngage = hardwareMap.servo.get("indexEngage");

        intake = hardwareMap.dcMotor.get("intake");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        magnet1 = hardwareMap.get(DigitalChannel.class, "mag1");
        magnet2 = hardwareMap.get(DigitalChannel.class, "mag2");
        magnet3 = hardwareMap.get(DigitalChannel.class, "mag3");

        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);
        hood = hardwareMap.servo.get("hood");
        lift = hardwareMap.servo.get("lift");
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
        if (gamepad1.share) {
            redAlliance = false;
        }
        if (gamepad1.options) {
            imu.resetYaw();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = Math.toRadians(180);
        double rotX = 1.1 * (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }
    public void index() {
        indexTimer++;
        if (gamepad1.rightBumperWasPressed()) intake();
        if (gamepad1.a) intakePower = intakeOut;
        if (gamepad1.aWasReleased()) intakePower = 0;
//        if (gamepad1.rightBumperWasReleased() || gamepad1.aWasReleased()) intakePower = 0;
        if (gamepad1.yWasPressed()) { // triangle on ps controller
            indexTimer = 0;
            indexState = Index.AUTO;
        }
        if (gamepad1.bWasPressed()) indexState = Index.FORWARD;
        if (gamepad1.xWasPressed()) indexState = Index.BACKWARD;
        if (gamepad1.bWasReleased() || gamepad1.xWasReleased()) indexState = Index.HOLD;
        switch (indexState) {

            case AUTO:
                engageIndex();
                autoIndex();
                break;

            case FORWARD:
                engageIndex();
                indexPower = correctIndex;
                break;

            case BACKWARD:
                engageIndex();
                indexPower = -correctIndex;
                break;

            case SHOOT:
                engageIndex();
                indexPower = 0.0;
                autoShoot();
                break;

            case HOLD:
                engageIndex();
                indexPower = 0.0;
                break;

            case INTAKE:
                disengageIndex();
                indexPower = 0.0;
                break;
        }
    }
    public void intake() {
        if (intakePower == intakeIn) intakePower = 0;
        else intakePower = intakeIn;
        indexState = Index.INTAKE;
    }
    public void outtake() {
        intakePower = intakeOut;
        indexState = Index.INTAKE;
    }
    public void engageIndex() {
        indexEngagePos = indexEngaged;
    }
    public void disengageIndex() {
        indexEngagePos = indexDisengaged;
    }
    public void autoIndex() {
        boolean mag1 = !magnet1.getState();
        boolean mag2 = !magnet2.getState();
        boolean mag3 = !magnet3.getState();
//        if (mag3) {
//            indexState = Index.SHOOT;
//        } else if (mag2) {
//            indexState = Index.FORWARD;
//        } else if (mag1) {
//            indexState = Index.BACKWARD;
//        }
        if (!gamepad1.y) {
            if (mag3) {
                if (mag2) indexPower = correctIndex;
                else if (mag1) {
                    gamepad1.rumble(250);
                    indexPower = 0;
                }
//                else indexState = Index.SHOOT;
            }
            else indexPower = autoIndex;
//            else if (mag1) indexPower = -correctIndex;
        }
        else indexPower = autoIndex;
    }
    public void shoot() {
        if (gamepad1.dpad_right) turretPower = turretManual;
        else if (gamepad1.dpad_left) turretPower = -turretManual;
        else turretPower = ll.fetchAlignment(limelight, redAlliance);
        if (turretPower != 6767) {
            if ((turretPower > 0 && turret.getCurrentPosition() < 200) || (turretPower < 0 && turret.getCurrentPosition() > -80)) {
                turret.setPower(turretPower); // Glenn 12/4/2025
            }
        } else turretPower = 0;
        hoodPos = (targetTps >= 1000 ? farHood : closeHood);
        if (gamepad1.right_trigger >= 0.2) {
            telemetry.setMsTransmissionInterval(limelightFast);
            targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
//            if (turretPower > 6766) gamepad1.rumble(200);
//            if ((turretPower > 0 && turret.getCurrentPosition() < 200) || (turretPower < 0 && turret.getCurrentPosition() > -80)) {
//                turret.setPower(turretPower); // Glenn 12/4/2025
//            } else turretPower = 0;
        } else if (gamepad1.dpad_up) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = closeSpeed;
            hoodPos = closeHood;
        } else if (gamepad1.dpad_down) {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = farSpeed;
            hoodPos = farHood;
        } else {
            telemetry.setMsTransmissionInterval(limelightSlow);
            targetTps = 0;
        }
        if (gamepad1.leftBumperWasPressed()) liftUp();
        if (gamepad1.leftBumperWasReleased()) liftDown();
    }
    public void autoShoot() {
        if (gamepad1.left_trigger < 0.2 && Math.abs(targetTps/thrower1.getVelocity()) <= 1.2 && Math.abs(turretPower) <= 0.05) liftUp();
    }
    public void liftUp() {
        liftPos = liftUp;
    }
    public void liftDown() {
        liftPos = liftDown;
    }
    public void powers() {

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        thrower1.setVelocity( -1 * targetTps); // Glenn 12/4/2025
        thrower2.setPower(thrower1.getPower());
        turret.setPower(turretPower); // Glenn 12/4/2025
        intake.setPower(intakePower);

        hood.setPosition(hoodPos);
        lift.setPosition(liftPos);
        indexEngage.setPosition(indexEngagePos);
        indexer.setPower(indexPower);
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