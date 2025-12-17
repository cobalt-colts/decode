package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.goalred;
import org.firstinspires.ftc.teamcode.util.subsystems;

import java.util.concurrent.Delayed;

@Autonomous(name = "NextFTC Red Goal")
public class goalrednextftc extends NextFTCOpMode {



    public goalrednextftc() throws InterruptedException {
        addComponents(
                new SubsystemComponent(subsystems.Lift.INSTANCE),
                new SubsystemComponent(subsystems.Thrower.INSTANCE),
                new SubsystemComponent(subsystems.Index.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(

                new ParallelGroup(
//                        subsystems.Thrower.INSTANCE.autoshootpos,
                        new FollowPath(goalred.launch1, true)
                ),
                subsystems.Index.INSTANCE.index3,
                new FollowPath(goalred.line1, false),
                subsystems.Intake.INSTANCE.intakeon,
                new FollowPath(goalred.line2, true),
                new FollowPath(goalred.launch2, true),
                new Delay(1),
                new FollowPath(goalred.offline, false)


        );
    }

    @Override public void onInit() {
        new ParallelGroup(
                subsystems.Index.INSTANCE.engage
        );
    }
    @Override
    public void onStartButtonPressed() {
        goalred.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(120, 128, Math.toRadians(216.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() { }

}