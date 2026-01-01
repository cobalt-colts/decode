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

@Configurable
@Config
@TeleOp(name="Lift Tests")
public class lifttest extends LinearOpMode {

    public static double lift1Pos = 0.5;
    public static double lift2Pos = 0.5;
    public static double lift3Pos = 0.5;


    /* TURRET MAX POS:
    0 IS OPPOSITE INTAKE
    -80 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor f1s1 = hardwareMap.get(ColorSensor.class, "f1s1");
        ColorSensor f1s2 = hardwareMap.get(ColorSensor.class, "f1s2");
        ColorSensor f2s1 = hardwareMap.get(ColorSensor.class, "f2s1");
        ColorSensor f2s2 = hardwareMap.get(ColorSensor.class, "f2s2");
        ColorSensor f3s1 = hardwareMap.get(ColorSensor.class, "f3s1");
        ColorSensor f3s2 = hardwareMap.get(ColorSensor.class, "f3s2");
        Servo lift1 = hardwareMap.servo.get("flicker1");
        Servo lift2 = hardwareMap.servo.get("flicker2");
        Servo lift3 = hardwareMap.servo.get("flicker3");
//        AnalogInput lift1Analog = hardwareMap.get(AnalogInput.class, "lift1Analog");
//        AnalogInput lift2Analog = hardwareMap.get(AnalogInput.class, "lift2Analog");
//        AnalogInput lift3Analog = hardwareMap.get(AnalogInput.class, "lift3Analog");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift1.setPosition(lift1Pos);
            lift2.setPosition(lift2Pos);
            lift3.setPosition(lift3Pos);
            telemetry.addData("11 green: ", f1s1.green());
            telemetry.addData("12 green: ", f1s2.green());
            telemetry.addData("21 green: ", f2s1.green());
            telemetry.addData("22 green: ", f2s2.green());
            telemetry.addData("31 green: ", f3s1.green());
            telemetry.addData("32 green: ", f3s2.green());
            telemetry.addData("11 blue: ", f1s1.blue());
            telemetry.addData("12 blue: ", f1s2.blue());
            telemetry.addData("21 blue: ", f2s1.blue());
            telemetry.addData("22 blue: ", f2s2.blue());
            telemetry.addData("31 blue: ", f3s1.blue());
            telemetry.addData("32 blue: ", f3s2.blue());
//            telemetry.addData("Lift1: ", lift1Analog.getVoltage());
//            telemetry.addData("Lift2: ", lift2Analog.getVoltage());
//            telemetry.addData("Lift3: ", lift3Analog.getVoltage());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
