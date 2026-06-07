package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.paths.RedFar.*;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.paths.RedFar;
import org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12;
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
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="RED Far")
public class redfar extends NextFTCOpMode {
    public Command autonomousRoutine() {
        return new SequentialGroupFixed(
            new ParallelGroup(
                    Intake.INSTANCE.intake,
                    new FollowPath(launch1)
            ),
            new Delay(3),
            Index.INSTANCE.launchIfBall(),
            Index.INSTANCE.launchIfBall(),
            new ParallelGroup(
                    Turret.INSTANCE.setPos(0),
                    new FollowPath(cycle)
            ),
            Intake.INSTANCE.outtake,
            new TurnTo(Angle.fromDeg(70)),
            Index.INSTANCE.launchIfBall(),
            Index.INSTANCE.launchIfBall(),
            new ParallelGroup(
                    Intake.INSTANCE.intake,
                    new FollowPath(farline)
            ),
            new TurnTo(Angle.fromDeg(60)),
            Intake.INSTANCE.outtake,
            Index.INSTANCE.launchIfBall(),
            Index.INSTANCE.launchIfBall(),
            Intake.INSTANCE.intake,
            new FollowPath(cycle),
            Intake.INSTANCE.outtake,
            new TurnTo(Angle.fromDeg(70)),
            Index.INSTANCE.launchIfBall(),
            Index.INSTANCE.launchIfBall(),
            new FollowPath(leave)
        );
    }
    public redfar() {
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
        positions.redAlliance = true;
        positions.autoTurret = false;
        subsystems.teleop = false;
        subsystems.start = false;
        subsystems.far = false;
        Index.INSTANCE.alldown.schedule();
        Turret.INSTANCE.setPos(30).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        subsystems.teleop = false;
        positions.autoTurret = false;
        if (Thrower.limelight != null) {
            Thrower.limelight.pipelineSwitch(2);
        }
        RedFar.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(82.5, 9.5, Math.toRadians(90)));

        autonomousRoutine().schedule();
    }

    @Override
    public void onStop() {
        Turret.captureAutoTurretPosition();
        subsystems.start = false;
    }


}
