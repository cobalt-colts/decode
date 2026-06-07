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
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.posConstants;
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
    private static final double MIN_ENABLED_TARGET_TPS = 100.0;

    public static PIDFController controller;

    public static double p = ShooterPIDConfig.kP, i = ShooterPIDConfig.kI, d = ShooterPIDConfig.kD, f = ShooterPIDConfig.kF;

    public static double hoodpos = 0.55;

    public static int targetVelocity = 0;

    public static int turretPos = 0;

    public static double atVelocityTolerance = subsystems.Thrower.AT_VEL_TOL;
    public static boolean atVelocity = false;
    public static double currentVelocity = 0;
    public static double currentRpm = 0;
    public static double velocityError = 0;
    public static double motorPower = 0;

    private DcMotorEx masterShootingSpeedMotor;
    private DcMotorEx slaveShootingSpeedMotor;

    private DcMotorEx intake;

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

    private final Button flickAll = button(() -> gamepad1.right_bumper)
            .whenBecomesTrue(subsystems.Index.INSTANCE.farunsortedlaunch);

    private final Button flickFar = button(() -> gamepad1.left_bumper)
            .whenBecomesTrue(subsystems.Index.INSTANCE.closeunsortedlaunch);

    private final Button flickOne = button(() -> gamepad1.dpad_down)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch1);

    @Override
    public void onInit() {
        ptelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        positions.redAlliance = false;
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.flyWheelCorrect = 0;
        atVelocity = false;

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPositionPIDFCoefficients(posConstants.turretP);

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        controller = new PIDFController(p, i, d, f);

        masterShootingSpeedMotor = hardwareMap.get(DcMotorEx.class, "thrower1");
        slaveShootingSpeedMotor = hardwareMap.get(DcMotorEx.class, "thrower2");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        hood = hardwareMap.servo.get("hood");

        hood.setPosition(hoodpos);

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

        syncShooterPidConfig();
        controller.setPIDF(p, i, d, f);

        currentVelocity = masterShootingSpeedMotor.getVelocity();
        currentRpm = (currentVelocity / ShooterPIDConfig.TICKS_PER_REV) * 60.0;
        velocityError = targetVelocity - currentVelocity;
        atVelocity = Math.abs(velocityError) <= atVelocityTolerance;

        double pid = controller.calculate(currentVelocity, targetVelocity);
        motorPower = Range.clip(pid, -1.0, 1.0);

        if (targetVelocity < MIN_ENABLED_TARGET_TPS) {
            motorPower = 0;
            atVelocity = false;
            stopShooter();
        } else {
            masterShootingSpeedMotor.setMotorEnable();
            slaveShootingSpeedMotor.setMotorEnable();
            masterShootingSpeedMotor.setPower(motorPower);
            slaveShootingSpeedMotor.setPower(motorPower);
        }

        turret.setTargetPosition(turretPos);
        turret.setPower(1);

        double commandedHoodPos = Math.clamp(hoodpos + subsystems.Index.INSTANCE.hoodoffset, 0, 1);
        hood.setPosition(commandedHoodPos);

        intake.setPower(-1);

        telemetry.addData("motorPower", motorPower);
        telemetry.addData("currentVelocity (ticks/sec)", currentVelocity);
        telemetry.addData("currentRpm", currentRpm);
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("velocityError", velocityError);
        telemetry.addData("atVelocity", atVelocity);
        telemetry.addData("hoodPos", commandedHoodPos);
        telemetry.addData("turretTarget", turretPos);
        telemetry.addData("real velo", subsystems.Thrower.targetvelocity + subsystems.Index.INSTANCE.veloffset);
        telemetry.update();

        ptelemetry.addData("current velocity", currentVelocity);
        ptelemetry.addData("target velocity", targetVelocity);
        ptelemetry.addData("velocity error", velocityError);
        ptelemetry.addData("at velocity", atVelocity);
        ptelemetry.addData("motor power", motorPower);
        ptelemetry.addData("hood position", commandedHoodPos);
        ptelemetry.addData("hood offset", subsystems.Index.INSTANCE.hoodoffset);
        ptelemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        stopShooter();
        if (intake != null) {
            intake.setPower(0);
        }
        subsystems.start = false;
        subsystems.teleop = false;
        atVelocity = false;
    }

    private static void syncShooterPidConfig() {
        ShooterPIDConfig.kP = p;
        ShooterPIDConfig.kI = i;
        ShooterPIDConfig.kD = d;
        ShooterPIDConfig.kF = f;
        subsystems.Thrower.AT_VEL_TOL = atVelocityTolerance;
    }

    private void stopShooter() {
        if (masterShootingSpeedMotor != null) {
            masterShootingSpeedMotor.setPower(0);
            masterShootingSpeedMotor.setMotorDisable();
        }
        if (slaveShootingSpeedMotor != null) {
            slaveShootingSpeedMotor.setPower(0);
            slaveShootingSpeedMotor.setMotorDisable();
        }
    }
}
