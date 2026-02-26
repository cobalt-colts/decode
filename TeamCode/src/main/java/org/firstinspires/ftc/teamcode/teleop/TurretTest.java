package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.posConstants.limelight;
import static org.firstinspires.ftc.teamcode.util.posConstants.limelightFast;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ll;
import org.firstinspires.ftc.teamcode.util.posConstants;

@Configurable
@Config
@TeleOp(name="Lift Tests")
@SuppressWarnings("CannotResolve")
public class TurretTest extends LinearOpMode {

    public static double rightFlickerPos = posConstants.flicker1down;
    public static double backFlickerPos = posConstants.flicker2down;
    public static double leftFlickerPos = posConstants.flicker3down;

    public static double power = 0;
    public static double turretPos = 0;
    public static boolean runTo = false;
    public static boolean red = true;


    /* TURRET MAX POS:
    0 IS MAGNET
    -100 TO 200
     */


    @Override
    public void runOpMode() throws InterruptedException {

        Servo rightFlicker = hardwareMap.servo.get("flicker1");
        Servo backFlicker = hardwareMap.servo.get("flicker2");
        Servo leftFlicker = hardwareMap.servo.get("flicker3");
        AnalogInput rightAnalog = hardwareMap.get(AnalogInput.class, "rightAnalog");
        AnalogInput backAnalog = hardwareMap.get(AnalogInput.class, "backAnalog");
        AnalogInput leftAnalog = hardwareMap.get(AnalogInput.class, "leftAnalog");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(limelightFast);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) rightFlickerPos = posConstants.flicker1up;
            if (gamepad1.aWasReleased()) rightFlickerPos = posConstants.flicker1down;
            rightFlicker.setPosition(rightFlickerPos);

            if (gamepad1.yWasPressed()) rightFlickerPos = posConstants.flicker2up;
            if (gamepad1.yWasReleased()) rightFlickerPos = posConstants.flicker2down;
            backFlicker.setPosition(backFlickerPos);

            if (gamepad1.xWasPressed()) rightFlickerPos = posConstants.flicker3up;
            if (gamepad1.xWasReleased()) rightFlickerPos = posConstants.flicker3down;
            leftFlicker.setPosition(leftFlickerPos);

            turretPos = turret.getCurrentPosition() + ll.fetchAlignment(limelight, red);

            turret.setTargetPosition((int) turretPos);
            turret.setPower(power);
            turret.setMode(runTo ? DcMotor.RunMode.RUN_TO_POSITION : DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("rightFlickerpos", rightFlicker.getPosition());
            telemetry.addData("backFlickerpos", backFlicker.getPosition());
            telemetry.addData("leftFlickerpos", leftFlicker.getPosition());
            telemetry.addData("rightFlicker: ", rightAnalog.getVoltage());
            telemetry.addData("backFlicker: ", backAnalog.getVoltage());
            telemetry.addData("leftFlicker: ", leftAnalog.getVoltage());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.update();

            TelemetryManager.TelemetryWrapper panelstel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

            panelstel.addData("rightFlickerpos", rightFlicker.getPosition());
            panelstel.addData("rightFlicker: ", rightAnalog.getVoltage());
            panelstel.addLine();
            panelstel.addData("backFlickerpos", backFlicker.getPosition());
            panelstel.addData("backFlicker: ", backAnalog.getVoltage());
            panelstel.addLine();
            panelstel.addData("leftFlickerpos", leftFlicker.getPosition());
            panelstel.addData("leftFlicker: ", leftAnalog.getVoltage());
            panelstel.addLine();
            panelstel.addLine();
            panelstel.addData("turret", turret.getCurrentPosition());
            panelstel.addData("Target", (int) turretPos);
            panelstel.addData("Error", (int) turretPos - turret.getCurrentPosition());
            panelstel.update();
        }
    }
}