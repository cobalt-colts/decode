package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.ll;

@Config
@TeleOp(name = "Shooter Dashboard Tuning", group = "Tuning")
public class ShooterDashboardTuning extends LinearOpMode {

    private DcMotorEx thrower1, thrower2;


    @Override
    public void runOpMode() throws InterruptedException {

        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo hood = hardwareMap.servo.get("hood");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            // Apply live Dashboard PIDF values
            thrower1.setVelocityPIDFCoefficients(
                    ShooterPIDConfig.kP,
                    ShooterPIDConfig.kI,
                    ShooterPIDConfig.kD,
                    ShooterPIDConfig.kF
            );

            thrower2.setVelocityPIDFCoefficients(
                    ShooterPIDConfig.kP,
                    ShooterPIDConfig.kI,
                    ShooterPIDConfig.kD,
                    ShooterPIDConfig.kF
            );

//            double targetTps = ShooterPIDConfig.targetRpm * ShooterPIDConfig.TICKS_PER_REV / 60.0;
//            double targetTps = ll.fetchFlywheelSpeed(limelight) * ShooterPIDConfig.TICKS_PER_REV / 60.0;

            double targetTps = ShooterPIDConfig.TICKS_PER_REV / 60.0;
            if (ShooterPIDConfig.autoAim) targetTps *= ll.fetchFlywheelSpeed(limelight);
            else targetTps *= ShooterPIDConfig.targetRpm;

            thrower1.setVelocity(targetTps);
            thrower2.setVelocity(targetTps);

            double v1 = thrower1.getVelocity() / ShooterPIDConfig.TICKS_PER_REV * 60.0;
            double v2 = thrower2.getVelocity() / ShooterPIDConfig.TICKS_PER_REV * 60.0;

            hood.setPosition(ShooterPIDConfig.hoodPos);

            TelemetryPacket p = new TelemetryPacket();
            p.put("target_tps", targetTps);
            p.put("thrower1_tps", v1);
            p.put("thrower2_tps", v2);
            p.put("hood: ", hood.getPosition());
            p.put("ll: ", ll.fetchFlywheelSpeed(limelight));
            dashboard.sendTelemetryPacket(p);

            telemetry.addData("Target (tps)", targetTps);
            telemetry.addData("Thrower1 (tps)", v1);
            telemetry.addData("Thrower2 (tps)", v2);
            telemetry.addData("ll: ", ll.fetchFlywheelSpeed(limelight));
            telemetry.addData("hood:", hood.getPosition());
            telemetry.update();
        }
    }
}
