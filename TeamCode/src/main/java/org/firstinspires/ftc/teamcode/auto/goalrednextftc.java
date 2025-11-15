package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.goalred;
import org.firstinspires.ftc.teamcode.util.subsystems;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class goalrednextftc extends NextFTCOpMode {



    public goalrednextftc() throws InterruptedException {
        addComponents(
                new SubsystemComponent(subsystems.Thrower.INSTANCE, subsystems.Lift.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        subsystems.Thrower.INSTANCE.autoshootpos,
                        new FollowPath(goalred.preload, true)
                ),
                subsystems.Lift.INSTANCE.liftup



        );
    }

    @Override public void onInit() {
        goalred.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(120.000, 128.000, Math.toRadians(216.5)));
        subsystems.Thrower.INSTANCE.throwerstart.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() { }

}