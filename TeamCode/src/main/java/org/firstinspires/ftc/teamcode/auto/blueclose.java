package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.paths.blueGoalLines12.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.paths.blueGoalLines12;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.*;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="blue Close")
public class blueclose extends NextFTCOpMode {
    private static final double APPROACH_SPEED = 0.90;
    private static final double LINE_SPEED = 0.78;
    private static final double RETURN_SPEED = 0.95;
    private static final double GATE_SPEED = 0.72;

    public Command autonomousRoutine() {
        return new SequentialGroupFixed(
                new ParallelGroup(
                        Intake.INSTANCE.intake
                ),
                new FollowPath(launch1),
                new Delay(.75),
                Index.INSTANCE.launchIfBall(),
                new ParallelGroup(
                        new FollowPath(line1),
                        Turret.INSTANCE.setPos(160)
                ),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new Delay(.75),
                                Intake.INSTANCE.outtake
                        ),
                        new FollowPath(launch2)
                ),
                Intake.INSTANCE.outtake,
                new Delay(.15),
                Index.INSTANCE.launchIfBall(),
                Intake.INSTANCE.intake,
                new FollowPath(gate),
//                new FollowPath(gate2, GATE_SPEED),
                new Delay(.5),
                new ParallelGroup(
                        new SequentialGroupFixed(
                                new Delay(.75),
                                Intake.INSTANCE.outtake
                        ),
                        new FollowPath(returnToLaunch)

                ),
                Index.INSTANCE.launchIfBall(),
                Intake.INSTANCE.intake,
                new ParallelGroup(
                        new FollowPath(line2),
                        Turret.INSTANCE.setPos(120)
                ),
                Intake.INSTANCE.outtake,
                new Delay(.15),
                Index.INSTANCE.launchIfBall(),
                Index.INSTANCE.launchIfBall()
        );
    }
    public blueclose() {
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
        Turret.INSTANCE.setPos(0).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        subsystems.teleop = false;
        positions.autoTurret = false;
        if (Thrower.limelight != null) {
            Thrower.limelight.pipelineSwitch(3);
        }
        blueGoalLines12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(21.5, 123, Math.toRadians(143)));

        autonomousRoutine().schedule();
    }

    @Override
    public void onStop() {
        subsystems.start = false;
    }


}
