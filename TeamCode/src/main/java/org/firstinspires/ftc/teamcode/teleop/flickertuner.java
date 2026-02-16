package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Configurable
@TeleOp
public class flickertuner extends LinearOpMode {

    public static double flicker1pos = 0.5;
    public static double flicker2pos = 0.5;
    public static double flicker3pos = 0.5;

    public void runOpMode() throws InterruptedException {

        Servo flicker1 = hardwareMap.servo.get("flicker1");
        Servo flicker2 = hardwareMap.servo.get("flicker2");
        Servo flicker3 = hardwareMap.servo.get("flicker3");

        waitForStart();
        while (opModeIsActive()) {

            flicker1.setPosition(flicker1pos);
            flicker2.setPosition(flicker2pos);
            flicker3.setPosition(flicker3pos);

        }
    }
}
