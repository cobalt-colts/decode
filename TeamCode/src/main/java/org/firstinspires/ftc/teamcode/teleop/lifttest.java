package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.posConstants;

@Disabled
@Configurable
@Config
@TeleOp(name="Lift Tests")
@SuppressWarnings("CannotResolve")
public class lifttest extends LinearOpMode {

    public static double rightFlickerPos = posConstants.rightFlickerDown;
    public static double backFlickerPos = posConstants.backFlickerDown;
    public static double leftFlickerPos = posConstants.leftFlickerDown;


    /* TURRET MAX POS:
    0 IS OPPOSITE INTAKE
    -80 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

//        ColorSensor f1s1 = hardwareMap.get(ColorSensor.class, "f1s1");
//        ColorSensor f1s2 = hardwareMap.get(ColorSensor.class, "f1s2");
//        ColorSensor f2s1 = hardwareMap.get(ColorSensor.class, "f2s1");
//        ColorSensor f2s2 = hardwareMap.get(ColorSensor.class, "f2s2");
//        ColorSensor f3s1 = hardwareMap.get(ColorSensor.class, "f3s1");
//        ColorSensor f3s2 = hardwareMap.get(ColorSensor.class, "f3s2");
        Servo rightFlicker = hardwareMap.servo.get("flicker1");
        Servo backFlicker = hardwareMap.servo.get("flicker2");
        Servo leftFlicker = hardwareMap.servo.get("flicker3");
        AnalogInput rightAnalog = hardwareMap.get(AnalogInput.class, "rightAnalog");
        AnalogInput backAnalog = hardwareMap.get(AnalogInput.class, "backAnalog");
//        AnalogInput leftFlickerAnalog = hardwareMap.get(AnalogInput.class, "leftFlickerAnalog");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            rightFlicker.setPosition(rightFlickerPos);
            backFlicker.setPosition(backFlickerPos);
            leftFlicker.setPosition(leftFlickerPos);
            telemetry.addData("rightFlickerpos", rightFlicker.getPosition());
            telemetry.addData("backFlickerpos", backFlicker.getPosition());
            telemetry.addData("leftFlickerpos", leftFlicker.getPosition());



//            telemetry.addData("11 green: ", f1s1.green());
//            telemetry.addData("12 green: ", f1s2.green());
//            telemetry.addData("21 green: ", f2s1.green());
//            telemetry.addData("22 green: ", f2s2.green());
//            telemetry.addData("31 green: ", f3s1.green());
//            telemetry.addData("32 green: ", f3s2.green());
//            telemetry.addData("11 blue: ", f1s1.blue());
//            telemetry.addData("12 blue: ", f1s2.blue());
//            telemetry.addData("21 blue: ", f2s1.blue());
//            telemetry.addData("22 blue: ", f2s2.blue());
//            telemetry.addData("31 blue: ", f3s1.blue());
//            telemetry.addData("32 blue: ", f3s2.blue());
            telemetry.addData("rightFlicker: ", rightAnalog.getVoltage());
            telemetry.addData("backFlicker: ", backAnalog.getVoltage());
//            telemetry.addData("leftFlicker: ", leftFlickerAnalog.getVoltage());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
