package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

//import static org.firstinspires.ftc.teamcode.util.posConstants.*;
//import static org.firstinspires.ftc.teamcode.util.positions.*;
import  static org.firstinspires.ftc.teamcode.util.teleSubsystems.*;

@Disabled
@Config
@Configurable
@TeleOp(name = "Colors")
public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {



        ColorSensor right1 = hardwareMap.get(ColorSensor.class, "right1");
//        ColorSensor right2 = hardwareMap.get(ColorSensor.class, "right2");
        ColorSensor back1 = hardwareMap.get(ColorSensor.class, "back1");
        ColorSensor back2 = hardwareMap.get(ColorSensor.class, "back2");
        ColorSensor left1 = hardwareMap.get(ColorSensor.class, "left1");
//        ColorSensor left2 = hardwareMap.get(ColorSensor.class, "left2");



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("\n");
            telemetry.addData("Right1 green: ", right1.green());
            telemetry.addData("Right1 blue: ", right1.blue());
            telemetry.addData("right: ", getColor(right1, right1, 0));
//            telemetry.addData("Right2 green: ", right2.green());
//            telemetry.addData("Right2 blue: ", right2.blue());
            telemetry.addLine("\n");
            telemetry.addData("Back1 green: ", back1.green());
            telemetry.addData("Back1 blue: ", back1.blue());
            telemetry.addData("Back2 green: ", back2.green());
            telemetry.addData("Back2 blue: ", back2.blue());
            telemetry.addData("back: ", getColor(back1, back2, 0));
            telemetry.addLine("\n");
            telemetry.addData("Left1 green: ", left1.green());
            telemetry.addData("Left1 blue: ", left1.blue());
            telemetry.addData("left: ", getColor(left1, left1, 0));
//            telemetry.addData("Left2 green: ", left2.green());
//            telemetry.addData("Left2 blue: ", left2.blue());
            telemetry.update();
        }

    }

}