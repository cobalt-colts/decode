package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.io.Serializable;

//import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain.DriveClass;
//import org.firstinspires.ftc.teamcode.SubSystems.IntakeSystem.IntakeClass;
//import org.firstinspires.ftc.teamcode.SubSystems.ShootingSystem.ShootingAngle.HoodAngleClass;
//import org.firstinspires.ftc.teamcode.SubSystems.ShootingSystem.TransferWheel.TransferWheelClass;

@TeleOp
@Config
@Configurable
public class ShootingSpeedTuning extends LinearOpMode {

    public static PIDFController controller;

    public static double p = 0.005 ,i = 0 ,d = 0, f = 0.0004;

    public static double hoodpos = .5;

    public static int targetVelocity = 0;

    private static DcMotorEx masterShootingSpeedMotor;
    private static DcMotorEx slaveShootingSpeedMotor;

    private static Servo hood;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(p,i,d,f);

//        TransferWheelClass.init(hardwareMap);
//        IntakeClass.init(hardwareMap);
//        DriveClass.init(hardwareMap);
//        HoodAngleClass.init(hardwareMap);
//        HoodAngleClass.test(gamepad1);



        telemetry = new MultipleTelemetry(telemetry , FtcDashboard.getInstance().getTelemetry());

        masterShootingSpeedMotor = hardwareMap.get(DcMotorEx.class , "thrower1");
        slaveShootingSpeedMotor = hardwareMap.get(DcMotorEx.class , "thrower2");

        hood = hardwareMap.servo.get("hood");

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        masterShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        slaveShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while (opModeIsActive())
        {
            controller.setPIDF(p,i,d,f);

            double currentVelocity = masterShootingSpeedMotor.getVelocity(); // /28)*60

            double pid = controller.calculate(currentVelocity , targetVelocity);

            double power = pid;

            if(targetVelocity < 100)
            {
                masterShootingSpeedMotor.setMotorDisable();
                slaveShootingSpeedMotor.setMotorDisable();
            }
            else {
                masterShootingSpeedMotor.setPower(power);
                slaveShootingSpeedMotor.setPower(power);
            }

            hood.setPosition(hoodpos);

//            DriveClass.arcade(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


            telemetry.addData("motorPower:" , masterShootingSpeedMotor.getPower()*1000);
            telemetry.addData("currentVelocity:" ,currentVelocity);
            telemetry.addData("targetVelocity" , targetVelocity);
            telemetry.addData("currentError:", (targetVelocity-currentVelocity));
            telemetry.update();
        }
    }
}