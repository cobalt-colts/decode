package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.bluefarcorner;
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
@Autonomous(name = "CORNER blue Far-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "blue Far")
public class CORNERM3BlueFar extends NextFTCOpMode {



    public CORNERM3BlueFar() throws InterruptedException {
        addComponents(
                new SubsystemComponent(subsystems.Thrower.INSTANCE, subsystems.Index.INSTANCE, subsystems.Intake.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
//                subsystems.Thrower.INSTANCE.farshoot,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.25),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                Turret.INSTANCE.bluefar,
                new FollowPath(bluefarcorner.line1, true),
                new Delay(0.25),
                new FollowPath(bluefarcorner.launch1, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(bluefarcorner.line2, false),
                new FollowPath(bluefarcorner.bump1, false),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new FollowPath(bluefarcorner.bump2, true)
                ),
                new Delay(0.5),
                new FollowPath(bluefarcorner.launch2, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(bluefarcorner.offline, true)


//                new FollowPath(bluefar.line2, true),
//                new FollowPath(bluefar.launch2, true),
//                new FollowPath(bluefar.offline, true)


        );
    }

    @Override public void onInit() {
        subsystems.Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.blueFarInit;
        Turret.INSTANCE.bluefarinit.schedule();
        subsystems.far = true;
//        Thrower.INSTANCE.farhood.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        bluefarcorner.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(60, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}