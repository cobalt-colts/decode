package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.subsystems;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
@Config
@Configurable
public class ShootingSpeedTuning extends NextFTCOpMode {

    public static PIDFController controller;

    public static double p = ShooterPIDConfig.kP, i = ShooterPIDConfig.kI, d = ShooterPIDConfig.kD, f = ShooterPIDConfig.kF;

    public static double hoodpos = 0.55;

    public static int targetVelocity = 0;

    public static int turretPos = 0;

    private DcMotorEx masterShootingSpeedMotor;
    private DcMotorEx slaveShootingSpeedMotor;

    private DcMotorEx intake;

    private Servo flicker1;

    private DcMotorEx turret;

    private TelemetryManager.TelemetryWrapper ptelemetry;

    private Servo hood;

    public ShootingSpeedTuning() {
        addComponents(
                new SubsystemComponent(
                        subsystems.Index.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    Button flickall = button(() -> gamepad1.right_bumper)
            .whenBecomesTrue(subsystems.Index.INSTANCE.farunsortedlaunch);

    Button flickfar = button(() -> gamepad1.left_bumper)
            .whenBecomesTrue(subsystems.Index.INSTANCE.closeunsortedlaunch);

    Button flick1 = button(() -> gamepad1.dpad_down)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch1);

    @Override
    public void onInit() {
        ptelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPositionPIDFCoefficients(150);

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        positions.redAlliance = false;
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.flyWheelCorrect = 0;

        controller = new PIDFController(p, i, d, f);

        masterShootingSpeedMotor = hardwareMap.get(DcMotorEx.class, "thrower1");
        slaveShootingSpeedMotor = hardwareMap.get(DcMotorEx.class, "thrower2");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        hood = hardwareMap.servo.get("hood");

        hood.setPosition(hoodpos);

        flicker1 = hardwareMap.servo.get("flicker1");

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        masterShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slaveShootingSpeedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        masterShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        slaveShootingSpeedMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        controller.setPIDF(p, i, d, f);

        double currentVelocity = masterShootingSpeedMotor.getVelocity();

        double pid = controller.calculate(currentVelocity, targetVelocity);

        double power = pid;

        if (targetVelocity < 100) {
            masterShootingSpeedMotor.setMotorDisable();
            slaveShootingSpeedMotor.setMotorDisable();
        } else {
            masterShootingSpeedMotor.setPower(power);
            slaveShootingSpeedMotor.setPower(power);
        }

        turret.setTargetPosition(turretPos);
        turret.setPower(1);

        hood.setPosition(hoodpos + subsystems.Index.INSTANCE.hoodoffset);
        // LUT based on error for offset in the future

        intake.setPower(-1);

        telemetry.addData("motorPower:", masterShootingSpeedMotor.getPower() * 1000);
        telemetry.addData("currentVelocity:", currentVelocity);
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("currentError:", (targetVelocity - currentVelocity));
        telemetry.addData("\"real\" velo:", subsystems.Thrower.targetvelocity + subsystems.Index.INSTANCE.veloffset);
        telemetry.update();
        ptelemetry.addData("current velocity", currentVelocity);
        ptelemetry.addData("target velocity", targetVelocity);
        ptelemetry.addData("hood offset", subsystems.Index.INSTANCE.hoodoffset);
        ptelemetry.update();
    }
}
