package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redfarcorner;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.*;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "CORNER RED Far-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Far")
public class CORNERM3RedFar extends NextFTCOpMode {



    public CORNERM3RedFar() throws InterruptedException {
        addComponents(
                new SubsystemComponent(subsystems.Thrower.INSTANCE, subsystems.Index.INSTANCE, subsystems.Intake.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
                subsystems.Thrower.INSTANCE.farshoot,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.25),
                subsystems.Index.INSTANCE.farunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),
                Turret.INSTANCE.redfar,
                new WaitUntil(() -> Math.abs(Math.abs(Turret.turret.getCurrentPosition()) - Math.abs(Turret.turretTargetPos)) <= 1),
                new FollowPath(redfarcorner.line1, true),
                new Delay(0.25),
                new InstantCommand(() -> {Intake.negative = false;}),
                new FollowPath(redfarcorner.launch1, true),
                new Delay(1), //0.5
                Index.INSTANCE.farunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),
                new FollowPath(redfarcorner.line2, false),
                new FollowPath(redfarcorner.bump1, false),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new FollowPath(redfarcorner.bump2, true)
                ),
                new Delay(0.5),
                new FollowPath(redfarcorner.launch2, true),
                new Delay(1), //0.5
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(redfarcorner.offline, true)


//                new FollowPath(redfar.line2, true),
//                new FollowPath(redfar.launch2, true),
//                new FollowPath(redfar.offline, true)


        );
    }

    @Override public void onInit() {
        subsystems.Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.redFarInit;
        Turret.INSTANCE.redinit.schedule();
        Thrower.INSTANCE.farhood.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redfarcorner.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(84, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}