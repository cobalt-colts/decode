package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

import java.util.List;


@Config
@Configurable
@TeleOp(name = "Meet 1 TeleOp SAFE")
public class derekDecodeMainSAFE extends LinearOpMode {

    public static double flyWheelP = 0;
    public static double flyWheelI = 0;
    public static double flyWheelD = 0;
    public static double flyWheelF = 0;
    public static double flyWheelPower = 6000;
    public static double intakePower = 0.75;

    public static double indexerPos = 0.93;
    public static double indexerPower = 0;

    public static double liftPos = 1;
    public static boolean liftUp = false;
    public static boolean engageIndex = false;

    public static boolean moveIndex = false;

    public static boolean indexeractive = false;

    double targetTps = 0;

    public static double engageopenpos = 0.7;
    public static double liftpos = 1;
    public static double hoodpos = 0.35; // bottom is 0.35, top is 0.

    final double indexerHome = 0.93;
    final double indexerPush1 = 0.85;
    final double indexerPush2 = 0.80;
    final double indexerPush3 = 0.75;

    final double indexerTransfer = 0.75;

    double rx = 0;

    private boolean isWhite(NormalizedColorSensor sensor){
        NormalizedRGBA colors = sensor.getNormalizedColors();
        return colors.red >= 0.04 && colors.blue >= 0.04 && colors.green >= 0.04;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower1.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF
        );

        thrower2.setVelocityPIDFCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD, ShooterPIDConfig.kF);


        DcMotor intake = hardwareMap.dcMotor.get("intake");
        CRServo indexer = hardwareMap.crservo.get("indexer");
        Servo indexengage = hardwareMap.servo.get("indexEngage");

        Servo lift = hardwareMap.servo.get("lift");
        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        NormalizedColorSensor indexsensor = hardwareMap.get(NormalizedColorSensor.class, "indexSensor");


        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0);

        limelight.stop();


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();



        if (isStopRequested()) return;

        // State machine for indexer
        int indexerState = 0; // 0: idle, 1: moving to white, 2: moving past white, 3: moving to next white

        indexengage.setPosition(engageopenpos);

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            if (!gamepad1.left_bumper) rx = -gamepad1.right_stick_x;

            limelight.start();

//            if (gamepad1.left_bumper) {
//                LLResult result = limelight.getLatestResult();
//                if(result != null && result.isValid()) {
//                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//                    double horizontalOffset = result.getTx();
//                    double turnPower = 0.25;
//                    double tolerance = 1;
//
//                    if (Math.abs(horizontalOffset) > tolerance) {
//                        if (horizontalOffset > 0) {
//                            rx = -turnPower;
//
//                        } else {
//                            rx = turnPower;
//                        }
//                    } else {
//                        frontRightMotor.setPower(0);
//                        frontLeftMotor.setPower(0);
//                        backLeftMotor.setPower(0);
//                        backRightMotor.setPower(0);
//                    }
//                }
//            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.right_bumper) {
//                indexerState = 0;
                intake.setPower(1);
                engageIndex = false;
            } else if (gamepad1.cross) {
                intake.setPower(-0.4);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.dpad_left) hoodpos = 0.35;
            else hoodpos = 0.25;

            // Start the indexer sequence
            if (gamepad1.triangle) {
//                indexerState = 1;
                engageIndex = true;
                moveIndex = true;
            }

            if (gamepad1.circle) {
                engageIndex = true;
                indexerPower =.25;
            } else if (gamepad1.square) {
                engageIndex = true;
                indexerPower = -.25;
            } else if (moveIndex) {
                if (isWhite(indexsensor)) {
                    indexerPower = 0;
                    moveIndex = false;
                }
                else {
                    engageIndex = true;
                    indexerPower = 0.125;
                }
            } else {
//                engageIndex = false;
                indexerPower = 0;
            }

            // Indexer State Machine
//            switch (indexerState) {
//                case 1: // Move until white is detected
//                    if (isWhite(indexsensor)) {
//                        indexerState = 2;
//                    } else {
//                        indexerState = 4;
//                    }
//                    break;
//                case 2: // Move until white is no longer detected
//                    if (!isWhite(indexsensor)) {
//                        indexerState = 3;
//                    } else {
//                        indexer.setPower(0.125);
//                        intake.setPower(1);
//                    }
//                    break;
//                case 3: // Move until the next white is detected
//                    if (isWhite(indexsensor)) {
//                        indexer.setPower(0);
//                        indexerState = 0; // Sequence finished
//                    } else {
//                        indexer.setPower(0.125);
//                        intake.setPower(1);
//                    }
//                    break;
//                case 4:
//                    if (isWhite(indexsensor)) {
//                        indexer.setPower(0);
//                        indexerState = 0; // Sequence finished
//                    } else {
//                        indexer.setPower(0.125);
//                        intake.setPower(1);
//
//                    }
//                    break;
//
//                default: // Idle state
//                    indexer.setPower(0);
//                    break;
//            }

            if (gamepad1.right_trigger >= 0.2) {
                targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;
                if (targetTps >= 1000) hoodpos = .15;
                else hoodpos = .25;

                if (targetTps < 775) gamepad1.rumble(500);

                double v1 = thrower1.getVelocity();
                double v2 = thrower2.getVelocity();

                if (Math.abs(targetTps/thrower1.getVelocity()) <= 1.3 && !gamepad1.dpad_down && !gamepad1.left_bumper) liftUp = true;


//                telemetry.addData("error", Math.abs(targetTps - thrower1.getVelocity()) <= 5 && !gamepad1.dpad_down);
//                telemetry.addData("Target (tps)", targetTps);
//                telemetry.addData("Thrower1 (tps)", v1);
//                telemetry.addData("Thrower2 (tps)", v2);
//                telemetry.update();
//                hood.setPosition(ll.fetchHoodPos(limelight));
//                hoodPos = 0.25;

                if (gamepad1.left_bumper) {
                    LLResult result = limelight.getLatestResult();
                    if(result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        double horizontalOffset = -result.getTy();
                        double turnPower = 0.25;
                        double tolerance = 1;

                        if (Math.abs(horizontalOffset) > tolerance) {
                            if (horizontalOffset > 0) {
                                frontLeftMotor.setPower(-turnPower);
                                frontRightMotor.setPower(turnPower);
                                backLeftMotor.setPower(-turnPower);
                                backRightMotor.setPower(turnPower);

                            } else {
                                frontLeftMotor.setPower(turnPower);
                                frontRightMotor.setPower(-turnPower);
                                backLeftMotor.setPower(turnPower);
                                backRightMotor.setPower(-turnPower);
                            }
                        } else {
                            frontRightMotor.setPower(0);
                            frontLeftMotor.setPower(0);
                            backLeftMotor.setPower(0);
                            backRightMotor.setPower(0);
                        }
                    }
                }
            } else {
                targetTps = 0;
            }

            if (gamepad1.dpad_up){
                liftUp = true;
            } else if (gamepad1.dpad_down || gamepad1.right_trigger < 0.1){
                liftUp = false;
            }

            hood.setPosition(hoodpos);
            thrower1.setVelocity(targetTps);
            thrower2.setVelocity(targetTps);
            lift.setPosition(liftUp ? 0 : 0.9);
            indexengage.setPosition(engageIndex ? 0.825 : 0.7);
            indexer.setPower(indexerPower);

            // Telemetry
            NormalizedRGBA colors = indexsensor.getNormalizedColors();
            telemetry.addData("error: ", targetTps/thrower1.getVelocity());
            telemetry.addData("red", colors.red);
            telemetry.addData("green", colors.green);
            telemetry.addData("blue", colors.blue);
            telemetry.addData("hood: ", hoodpos);
            telemetry.addData("target: ", targetTps);
            telemetry.addData("thrower1velocity", thrower1.getVelocity(AngleUnit.DEGREES)*60);
            telemetry.addData("thrower2velocity", thrower2.getVelocity(AngleUnit.DEGREES)*60);
            telemetry.update();

        }
    }
}
