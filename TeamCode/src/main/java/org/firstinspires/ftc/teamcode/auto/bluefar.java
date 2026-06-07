package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.paths.BlueFar.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.paths.BlueFar;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Camera;
import org.firstinspires.ftc.teamcode.util.subsystems.ColorSensing;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;
import org.firstinspires.ftc.teamcode.util.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="BLUE Far")
public class bluefar extends NextFTCOpMode {

    public Command autonomousRoutine() {
        return new SequentialGroupFixed(
                new ParallelGroup(
                        Intake.INSTANCE.intake,
                        new FollowPath(launch1)
                ),
                new Delay(3),
                Index.INSTANCE.farLaunchIfBall(),
                Index.INSTANCE.farLaunchIfBall(),
                new ParallelGroup(
                        Turret.INSTANCE.setPos(0),
                        new FollowPath(cycle)
                ),
                Intake.INSTANCE.outtake,
                new ParallelRaceGroup(
                        new Delay(1),
                        new TurnBy(Angle.fromDeg(-73))
                ),
                Index.INSTANCE.farLaunchIfBall(),
                Index.INSTANCE.farLaunchIfBall(),
                new ParallelGroup(
                        Intake.INSTANCE.intake,
                        new FollowPath(farline)
                ),
                Intake.INSTANCE.outtake,
                new ParallelRaceGroup(
                        new Delay(1),
                        new TurnTo(Angle.fromDeg(95))
                ),
                Index.INSTANCE.farLaunchIfBall(),
                Index.INSTANCE.farLaunchIfBall(),
                Intake.INSTANCE.intake,
                new FollowPath(cycle),
                Intake.INSTANCE.outtake,
                new ParallelRaceGroup(
                        new Delay(1),
                        new TurnBy(Angle.fromDeg(-73))
                ),
                Index.INSTANCE.farLaunchIfBall(),
                Index.INSTANCE.farLaunchIfBall(),
                new FollowPath(leave)
        );
    }
    public bluefar() {
        subsystems.teleop = false;
        subsystems.start = false;
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        Thrower.INSTANCE,
                        Index.INSTANCE,
                        Intake.INSTANCE,
                        Turret.INSTANCE,
                        Camera.INSTANCE,
                        ColorSensing.INSTANCE
                )
        );
    }

    @Override
    public void onInit() {
        positions.redAlliance = false;
        positions.autoTurret = false;
        subsystems.teleop = false;
        subsystems.start = false;
        subsystems.far = false;
        Index.INSTANCE.alldown.schedule();
        Turret.INSTANCE.setPos(-30).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        subsystems.teleop = false;
        positions.autoTurret = false;
        if (Thrower.limelight != null) {
            Thrower.limelight.pipelineSwitch(3);
        }
        BlueFar.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(59, 9.5, Math.toRadians(90)));
        autonomousRoutine().schedule();
    }

    @Override
    public void onStop() {
        Turret.captureAutoTurretPosition();
        subsystems.start = false;
    }


}
