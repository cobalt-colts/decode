package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp(name = "RED COWTOWN TeleOp")
public class milestele extends NextFTCOpMode {

    private boolean teleopPipelineConfigured = false;

    public milestele() throws InterruptedException {
        positions.redAlliance = true;
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
        positions.redAlliance = true;
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.autoTurret = true;
        positions.flyWheelCorrect = 0;
    }

    @Override
    public void onStartButtonPressed() {
        positions.redAlliance = true;
        PedroComponent.Companion.follower().setStartingPose(new Pose(0, 0, Math.toRadians(redTeleopForwardHeadingDeg)));
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                () -> subsystems.teleopDrivePower(gamepad1.left_stick_y, gamepad1.left_stick_x),
                () -> subsystems.teleopStrafePower(gamepad1.left_stick_y, gamepad1.left_stick_x),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();
        subsystems.start = true;
        subsystems.teleop = true;
        positions.autoTurret = true;
        if (subsystems.Thrower.limelight != null) {
            subsystems.Thrower.limelight.pipelineSwitch(redTeleopPipeline);
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
            subsystems.Thrower.limelight.pipelineSwitch(redTeleopPipeline);
            teleopPipelineConfigured = true;
        }
        subsystems.updateTeleopLimelightOrientation();
        ActiveOpMode.telemetry().addData("intake", gamepad1.left_bumper ? "OUT" : "IN");
        ActiveOpMode.telemetry().addData("alliance", positions.redAlliance ? "RED" : "BLUE");
        ActiveOpMode.telemetry().addData("forward heading", redTeleopForwardHeadingDeg);
        ActiveOpMode.telemetry().addData("robot heading deg", subsystems.lastRobotHeadingDeg);
        ActiveOpMode.telemetry().addData("ll tx", subsystems.lastLimelightTxDeg);
        ActiveOpMode.telemetry().addData("ll aim valid", subsystems.lastLimelightAimValid);
        ActiveOpMode.telemetry().addData("ll aim stale ms", subsystems.lastLimelightAimStalenessMs);
        ActiveOpMode.telemetry().addData("ll aim bearing", subsystems.lastLimelightAimBearingDeg);
        ActiveOpMode.telemetry().addData("turret aim raw deg", subsystems.lastTurretAimDegRaw);
        ActiveOpMode.telemetry().addData("turret aim deg", subsystems.lastTurretAimDeg);
        ActiveOpMode.telemetry().addData("turret logical ticks", subsystems.lastTurretLogicalPosition);
        ActiveOpMode.telemetry().addData("turret encoder offset", subsystems.lastTurretEncoderOffsetTicks);
        ActiveOpMode.telemetry().addData("turret target ticks", subsystems.lastTurretTargetPos);
        ActiveOpMode.telemetry().addData("turret motor target", subsystems.lastTurretMotorTargetPos);
        ActiveOpMode.telemetry().addData("magnet raw", subsystems.lastMagnetRawState);
        ActiveOpMode.telemetry().addData("magnet triggered", subsystems.lastMagnetTriggered);
        ActiveOpMode.telemetry().addData("hood vel error rpm", subsystems.Thrower.lastVelocityErrorRpm);
        ActiveOpMode.telemetry().addData("hood vel error filtered", subsystems.Thrower.lastVelocityErrorFilteredRpm);
        ActiveOpMode.telemetry().addData("hood vel offset", subsystems.Thrower.lastVelocityHoodOffset);
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
