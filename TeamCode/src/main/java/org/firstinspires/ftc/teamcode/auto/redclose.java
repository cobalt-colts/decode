package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.*;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="RED Close")
public class redclose extends NextFTCOpMode {

    public Command autonomousRoutine() {
        return new SequentialGroupFixed(
                new ParallelGroup(
                        Intake.INSTANCE.intake
                ),
                new FollowPath(launch1),
                new ParallelGroup(
                        new FollowPath(line1),
                        Turret.INSTANCE.setPos(-120)
                )
        );
    }
    public redclose() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        Thrower.INSTANCE,
                        Intake.INSTANCE,
                        Turret.INSTANCE
                )
        );
    }

    @Override
    public void onInit() {
        positions.redAlliance = true;
        positions.autoTurret = false;
        subsystems.teleop = false;
        subsystems.start = false;
        Turret.INSTANCE.setPos(0).schedule();
    }

    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        subsystems.teleop = false;
        positions.autoTurret = false;
        if (Thrower.limelight != null) {
            Thrower.limelight.pipelineSwitch(2);
        }
        RedGoalLines12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(120, 123, Math.toRadians(37)));

       autonomousRoutine().schedule();
    }

    @Override
    public void onStop() {
        subsystems.start = false;
    }


}
