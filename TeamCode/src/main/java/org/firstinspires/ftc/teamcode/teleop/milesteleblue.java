package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.subsystems;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog
@TeleOp(name = "BLUE COWTOWN TeleOp")
public class milesteleblue extends NextFTCOpMode {

    private boolean teleopPipelineConfigured = false;

    private Command autoPark() {
        PathChain park = PedroComponent.Companion.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                PedroComponent.Companion.follower().getPose(),
                                new Pose(positions.redAlliance ? 38 : 103.5, 33)
                        )
                )
                .setLinearHeadingInterpolation(PedroComponent.Companion.follower().getHeading(), Math.toRadians(positions.redAlliance ? 180 : 0))
                .build();

        return new FollowPath(park);
    }

    public milesteleblue() throws InterruptedException {
        positions.redAlliance = false;
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.flyWheelCorrect = 0;
        addComponents(
                new SubsystemComponent(subsystems.Thrower.INSTANCE,
                        subsystems.Index.INSTANCE,
                        subsystems.Intake.INSTANCE,
                        subsystems.Turret.INSTANCE,
                        subsystems.Camera.INSTANCE,
                        subsystems.ColorSensing.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private GoBildaPinpointDriver pinpoint;

    Button reset = button(() -> gamepad1.options)
            .whenBecomesTrue(() -> {
                subsystems.setTeleopForwardToCurrentHeading();
            });

    Button flick1 = button(() -> gamepad1.dpad_left)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch1);

    Button flick2 = button(() -> gamepad1.dpad_up)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch2);

    Button flick3 = button(() -> gamepad1.dpad_right)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch3);

    Button park = button(() -> gamepad1.dpad_down)
            .whenBecomesTrue(autoPark());

    Button flickall = button(() -> gamepad1.right_bumper)
            .whenBecomesTrue(() -> subsystems.Index.INSTANCE.launchPresentOnce().schedule());

    Button altflick = button(() -> gamepad1.right_trigger_pressed)
            .whenBecomesTrue(() -> subsystems.Index.INSTANCE.closeunsortedlaunch.schedule());
//
//    Button shooterBoost = button(() -> gamepad1.touchpad)
//            .whenBecomesTrue(() -> positions.flyWheelCorrect = 100);
//
//    Button shooterTrimDown = button(() -> gamepad1.right_stick_button)
//            .whenBecomesTrue(() -> positions.flyWheelCorrect -= 50);
//
//    Button shooterTrimReset = button(() -> gamepad1.left_stick_button)
//            .whenBecomesTrue(() -> positions.flyWheelCorrect = 0);

    @Override
    public void onInit() {
        positions.redAlliance = false;
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.autoTurret = true;
        positions.flyWheelCorrect = 0;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void onStartButtonPressed() {
        positions.redAlliance = false;
        PedroComponent.Companion.follower().setStartingPose(new Pose(0, 0, Math.toRadians(blueTeleopForwardHeadingDeg)));
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                () -> subsystems.teleopDrivePower(Gamepads.gamepad1().leftStickY().get(), Gamepads.gamepad1().leftStickX().get()),
                () -> subsystems.teleopStrafePower(Gamepads.gamepad1().leftStickY().get(), Gamepads.gamepad1().leftStickX().get()),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();
        subsystems.start = true;
        subsystems.teleop = true;
        positions.autoTurret = true;
        if (subsystems.Thrower.limelight != null) {
            subsystems.Thrower.limelight.pipelineSwitch(blueTeleopPipeline);
            teleopPipelineConfigured = true;
        }
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        if (!subsystems.start || !subsystems.teleop) {
            subsystems.Intake.INSTANCE.intakeMotor.setPower(0);
        } else if (gamepad1.left_bumper) {
            subsystems.Intake.INSTANCE.intakeMotor.setPower(0.5);
        } else {
            subsystems.Intake.INSTANCE.intakeMotor.setPower(-1);
        }

        if (!teleopPipelineConfigured && subsystems.Thrower.limelight != null) {
            subsystems.Thrower.limelight.pipelineSwitch(blueTeleopPipeline);
            teleopPipelineConfigured = true;
        }
        subsystems.updateTeleopLimelightOrientation();
        ActiveOpMode.telemetry().addData("intake", gamepad1.left_bumper ? "OUT" : "IN");
        ActiveOpMode.telemetry().addData("alliance", positions.redAlliance ? "RED" : "BLUE");
        ActiveOpMode.telemetry().addData("forward heading", blueTeleopForwardHeadingDeg);
        ActiveOpMode.telemetry().addData("turret target pos", subsystems.Turret.turretTargetPos);
        ActiveOpMode.telemetry().addData("turret current pos", subsystems.Turret.getLogicalCurrentPosition());
        ActiveOpMode.telemetry().addData("pinpoint heading", pinpoint.getHeading(AngleUnit.DEGREES));
        ActiveOpMode.telemetry().addData("target turret vel", subsystems.Thrower.targetvelocity);
        ActiveOpMode.telemetry().addData("ll tx", subsystems.lastLimelightTxDeg);
        ActiveOpMode.telemetry().addData("ll aim valid", subsystems.lastLimelightAimValid);
        ActiveOpMode.telemetry().addData("ll aim stale ms", subsystems.lastLimelightAimStalenessMs);
        ActiveOpMode.telemetry().addData("ll aim bearing", subsystems.lastLimelightAimBearingDeg);
        ActiveOpMode.telemetry().addData("turret aim deg", subsystems.lastTurretAimDeg);
        ActiveOpMode.telemetry().addData("localization", subsystems.lastLocalizationStatus);
        ActiveOpMode.telemetry().update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        subsystems.Intake.INSTANCE.intakeMotor.setPower(0);
        subsystems.start = false;
        subsystems.teleop = false;
        teleopPipelineConfigured = false;
    }
}
