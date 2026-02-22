package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.posConstants;

@Configurable
@Config
@TeleOp(name="Lift Tests")
@SuppressWarnings("CannotResolve")
public class TurretTest extends LinearOpMode {

    public static double rightFlickerPos = posConstants.flicker1down;
    public static double backFlickerPos = posConstants.flicker2down;
    public static double leftFlickerPos = posConstants.flicker3down;


    /* TURRET MAX POS:
    0 IS MAGNET
    -100 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

        Servo rightFlicker = hardwareMap.servo.get("flicker1");
        Servo backFlicker = hardwareMap.servo.get("flicker2");
        Servo leftFlicker = hardwareMap.servo.get("flicker3");
        AnalogInput rightAnalog = hardwareMap.get(AnalogInput.class, "rightAnalog");
        AnalogInput backAnalog = hardwareMap.get(AnalogInput.class, "backAnalog");
        AnalogInput leftFlickerAnalog = hardwareMap.get(AnalogInput.class, "leftFlickerAnalog");
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
            telemetry.addData("rightFlicker: ", rightAnalog.getVoltage());
            telemetry.addData("backFlicker: ", backAnalog.getVoltage());
            telemetry.addData("leftFlicker: ", leftFlickerAnalog.getVoltage());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}