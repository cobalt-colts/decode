package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.bluegoalsix;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "(SIX) BLUE Close-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Blue Goal")
public class SIXM3BlueGoal extends NextFTCOpMode {



    public SIXM3BlueGoal() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE, Index.INSTANCE, Intake.INSTANCE, subsystems.Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
                new ParallelGroup(
                        Thrower.INSTANCE.goalshoot,
                        new FollowPath(bluegoalsix.preload, true)
                ),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeunsortedlaunch,
                new Delay(0.15),
                subsystems.Turret.INSTANCE.bluegoal,
                new FollowPath(bluegoalsix.line1, true),
                new Delay(1), //0.5
                new InstantCommand(() -> {Intake.negative = false;}),
                new FollowPath(bluegoalsix.gate, true),
                new FollowPath(bluegoalsix.launch1, true),
                new Delay(0.75),
                Index.INSTANCE.closeunsortedlaunch,
                new FollowPath(bluegoalsix.offline, true)
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Thrower.INSTANCE.closehood.schedule();
//        Turret.initPos = 0;
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        subsystems.Turret.turretTargetPos = 0;
        bluegoalsix.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(26.5, 131.5, Math.toRadians(143.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}