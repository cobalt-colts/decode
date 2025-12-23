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
@TeleOp(name="Lift Test")
public class lifttest extends LinearOpMode {

    public static double lift1Pos = 0.25;
    public static double lift2Pos = 0.25;
    public static double lift3Pos = 0.25;


    /* TURRET MAX POS:
    0 IS OPPOSITE INTAKE
    -80 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor color11 = hardwareMap.get(ColorSensor.class, "color11");
        ColorSensor color12 = hardwareMap.get(ColorSensor.class, "color12");
        ColorSensor color21 = hardwareMap.get(ColorSensor.class, "color21");
        ColorSensor color22 = hardwareMap.get(ColorSensor.class, "color22");
        ColorSensor color31 = hardwareMap.get(ColorSensor.class, "color31");
        ColorSensor color32 = hardwareMap.get(ColorSensor.class, "color32");
        Servo lift1 = hardwareMap.servo.get("lift1");
        Servo lift2 = hardwareMap.servo.get("lift2");
        Servo lift3 = hardwareMap.servo.get("lift3");
        AnalogInput lift1Analog = hardwareMap.get(AnalogInput.class, "lift1Analog");
        AnalogInput lift2Analog = hardwareMap.get(AnalogInput.class, "lift2Analog");
        AnalogInput lift3Analog = hardwareMap.get(AnalogInput.class, "lift3Analog");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift1.setPosition(lift1Pos);
            lift2.setPosition(lift2Pos);
            lift3.setPosition(lift3Pos);
            telemetry.addData("11: ", color11.argb());
            telemetry.addData("12: ", color12.argb());
            telemetry.addData("21: ", color21.argb());
            telemetry.addData("22: ", color22.argb());
            telemetry.addData("31: ", color31.argb());
            telemetry.addData("32: ", color32.argb());
            telemetry.addData("Lift1: ", lift1Analog.getVoltage());
            telemetry.addData("Lift2: ", lift2Analog.getVoltage());
            telemetry.addData("Lift3: ", lift3Analog.getVoltage());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
