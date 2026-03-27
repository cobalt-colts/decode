package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
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

@TeleOp(name = "COWTOWN TeleOp")
public class milestele extends NextFTCOpMode {

    public milestele() throws InterruptedException {
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

    private Command resetDrive = new InstantCommand(() -> PedroComponent.Companion.follower().setPose(new Pose(0, 0, Math.toRadians(-180))));

    Button reset = button(() -> gamepad1.options)
            .whenBecomesTrue(resetDrive);

    Button intake = button(() -> gamepad1.left_bumper)
            .whenTrue(() -> {subsystems.Intake.INSTANCE.intakeMotor.setPower(0.5);})
            .whenFalse(() -> {subsystems.Intake.INSTANCE.intakeMotor.setPower(-1);});

    Button flick1 = button(() -> gamepad1.dpad_left)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch1);

    Button flick2 = button(() -> gamepad1.dpad_up)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch2);

    Button flick3 = button(() -> gamepad1.dpad_right)
            .whenBecomesTrue(subsystems.Index.INSTANCE.launch3);

    Button flickall = button(() -> gamepad1.dpad_down)
            .whenBecomesTrue(subsystems.Index.INSTANCE.closeunsortedlaunch);

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();
        PedroComponent.Companion.follower().setStartingPose(new Pose(0, 0, Math.toRadians(-180)));
        subsystems.start = true;
        subsystems.teleop = true;
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        ActiveOpMode.telemetry().addData("left trigger", intake.get());
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        subsystems.start = false;
    }
}
