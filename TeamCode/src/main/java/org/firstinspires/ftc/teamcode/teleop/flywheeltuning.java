package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@Config
@TeleOp
public class flywheeltuning extends LinearOpMode {

    public static double kP = 0;
    public static double kF = 0;

    public static double targetVelocity = 0;

    private double thrower2power = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            PIDFCoefficients pidf = new PIDFCoefficients(kP, 0, 0, kF);
            thrower1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

            thrower1.setVelocity(targetVelocity);

            if (thrower1.getPower() > 0.95) {
                thrower2power = 0.95;
            } else {
                thrower2power = thrower1.getPower();
            }

//            thrower2.setPower(thrower2power);

            telemetry.addData("Thrower 1 Velocity", thrower1.getVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Thrower Velocity", thrower1.getVelocity());
            packet.put("Target Velocuty", targetVelocity);
            packet.put("Baseline", 0);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}