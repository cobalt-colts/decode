package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Hood Reset")
public class hoodreset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.servo.get("hood");
        hood.setPosition(0);
        telemetry.addLine("Please place the hood on the servo, then press start.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            hood.setPosition(0.2);

        }
    }
}
