package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
@TeleOp(name="Lift Test")
public class lifttest extends LinearOpMode {

    public static double servopos = 0.25;


    /* TURRET MAX POS:
    0 IS OPPOSITE INTAKE
    -80 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

        Servo lift = hardwareMap.servo.get("lift");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift.setPosition(servopos);
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
