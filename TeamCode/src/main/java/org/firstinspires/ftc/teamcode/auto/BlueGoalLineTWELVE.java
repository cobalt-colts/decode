package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.BlueGoalLines12;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;

@Config
@Autonomous(name = "(12) BLUE Close-State", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "blue Goal")
public class BlueGoalLineTWELVE extends NextFTCOpMode {

    public BlueGoalLineTWELVE() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE,
                        Index.INSTANCE,
                        Intake.INSTANCE,
                        subsystems.Turret.INSTANCE,
                        subsystems.Camera.INSTANCE,
                        subsystems.ColorSensing.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Command autoRoutine() {
        return new SequentialGroupFixed(
                // Detect motif, drive to preload position, spin up shooter
                subsystems.Camera.INSTANCE.setmotif,
                new ParallelGroup(
                        Thrower.INSTANCE.shooteron,
                        new FollowPath(BlueGoalLines12.preload, true),
                        subsystems.Turret.INSTANCE.bluegoal
                ),

                // Shoot preloaded balls in motif order
                new InstantCommand(() -> Thrower.targetvelocity = 1370),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.closeSortedLaunch(),

                // Pick up line 1, drive to launch position, shoot
                Intake.INSTANCE.intake,
                new FollowPath(BlueGoalLines12.line1, true),
                new Delay(0.25),
                new FollowPath(BlueGoalLines12.gate, true),
                new FollowPath(BlueGoalLines12.launch1, true),
                new Delay(1),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeunsortedlaunch,
                Index.INSTANCE.sensedunsorted,

                // Pick up line 2, drive to launch position, shoot
                Intake.INSTANCE.intake,
                new FollowPath(BlueGoalLines12.line2, true),
                new Delay(0.5),
                new FollowPath(BlueGoalLines12.launch2, true),
                new Delay(1),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeunsortedlaunch,
                Index.INSTANCE.sensedunsorted,

                // Pick up line 3, drive to launch position, shoot
                subsystems.Turret.INSTANCE.bluepark,
                Intake.INSTANCE.intake,
                new FollowPath(BlueGoalLines12.line3, true),
                new Delay(0.5),
                new InstantCommand(() -> Thrower.targetvelocity = 1250),
                new FollowPath(BlueGoalLines12.launch3),
                new Delay(0.5),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeunsortedlaunch,
                Index.INSTANCE.sensedunsorted,
                Index.INSTANCE.sensedunsorted,
                Index.INSTANCE.sensedunsorted,

                // Return home
                subsystems.Turret.INSTANCE.home,
                new TurnTo(Angle.fromDeg(180))
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        subsystems.Turret.initPos = posConstants.blueGoalInit;
        subsystems.Turret.INSTANCE.bluegoalinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        BlueGoalLines12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(22, 126, Math.toRadians(324)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }
}