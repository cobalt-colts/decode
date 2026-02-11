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

@Config
@Configurable
@TeleOp(name = "Flywheel Tuning")
public class flywheeltuning extends LinearOpMode {

    // PIDF must be doubles
    public static double kp = 0.0;
    public static double kd = 0.0;
    public static double ff = 0.0;

    public static double targetRpm = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        // Ensure consistent motor configuration
        thrower1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        thrower2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        thrower1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        thrower2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        PIDFCoefficients pidf =
                new PIDFCoefficients(kp, 0.0, kd, ff);

        while (opModeIsActive()) {
            pidf.d = kd;
            pidf.p = kp;
            pidf.f = ff;

//            thrower1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
//            thrower1.setVelocityPIDFCoefficients(kp, 0, kd, ff);

            // Velocity control on both motors
//            thrower1.setVelocity(targetRpm);
            thrower1.setPower(0);
            thrower2.setPower(thrower1.getPower());

            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Thrower1 Position", thrower1.getCurrentPosition());
            telemetry.addData("Thrower1 Velocity", thrower1.getVelocity());
            telemetry.addData("Thrower1 Power", thrower1.getPower());
            telemetry.addData("Thrower2 Power", thrower2.getPower());
            telemetry.addData("pidf", thrower1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target_rpm", targetRpm);
            packet.put("thrower1_position", thrower1.getCurrentPosition());
            packet.put("thrower1_velocity", thrower1.getVelocity());

            dashboard.sendTelemetryPacket(packet);

        }
    }
}
