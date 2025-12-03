package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
@TeleOp(name="Lift Test")
public class lifttest extends LinearOpMode {

    public static double servopos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo lift = hardwareMap.servo.get("lift");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift.setPosition(servopos);
        }
    }
}
