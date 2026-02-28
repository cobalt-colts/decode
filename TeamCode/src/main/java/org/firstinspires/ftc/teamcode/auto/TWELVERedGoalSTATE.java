package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redgoalnine;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "(12) RED Close-State", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Goal")
public class TWELVERedGoalSTATE extends NextFTCOpMode {



    public TWELVERedGoalSTATE() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE, Index.INSTANCE, Intake.INSTANCE, subsystems.Turret.INSTANCE, subsystems.Camera.INSTANCE, subsystems.ColorSensing.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
                subsystems.Camera.INSTANCE.setmotif,
//                subsystems.Turret.INSTANCE.redgoal, miles, 1/16/25 (worked just fine w deafult paths, turret caused it to shoot outside field.)
                new ParallelGroup(
                        Thrower.INSTANCE.shooteron,
                        new FollowPath(redgoalnine.preload, true),
                        subsystems.Turret.INSTANCE.redgoal
                ),
                new InstantCommand(() -> {Thrower.targetvelocity = 1370;}),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.closeunsortedlaunch,
//                new IfElseCommand(
//                        () -> subsystems.hasAnyBalls,
//                        Index.INSTANCE.closesortedLaunch
//                ),
                Index.INSTANCE.closeunsortedlaunch,
//                Index.INSTANCE.secondcloseunsortedlaunch,
                Intake.INSTANCE.intake,

                new FollowPath(redgoalnine.line1, true),
//                subsystems.Turret.INSTANCE.redgoal,
                new Delay(0.25),
//                new InstantCommand(() -> {Intake.outtake();}),
                new FollowPath(redgoalnine.gate, true),
//                new Delay(0.5),
                new FollowPath(redgoalnine.launch1, true),
                new Delay(1), //0.5
//                Index.INSTANCE.flickerOrder,
//                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closesortedLaunch,
                Index.INSTANCE.closeunsortedlaunch,
//                new IfElseCommand(
//                        () -> subsystems.hasAnyBalls,
//                        Index.INSTANCE.closesortedLaunch
//                ),
                Index.INSTANCE.closeunsortedlaunch,
//                Index.INSTANCE.secondcloseunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(redgoalnine.line2, true),
                new Delay(0.5),
//                new InstantCommand(() -> {Intake.outtake();}),
                new FollowPath(redgoalnine.launch2, true),
                new Delay(1),
//                Index.INSTANCE.flickerOrder,
//                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closesortedLaunch,
                Index.INSTANCE.closeunsortedlaunch,
//                new IfElseCommand(
//                        () -> subsystems.hasAnyBalls,
//                        Index.INSTANCE.closesortedLaunch
//                ),
                Index.INSTANCE.closeunsortedlaunch,
//                Index.INSTANCE.secondcloseunsortedlaunch,
                new InstantCommand(subsystems.Turret.INSTANCE.redpark),
                Intake.INSTANCE.intake,
                new FollowPath(redgoalnine.line3, true),
                new Delay(0.5),
//                new InstantCommand(() -> {Intake.outtake();}),
                new InstantCommand(() -> {Thrower.targetvelocity = 1240;}),
                new FollowPath(redgoalnine.launch3),
                new Delay(0.5),
//                Index.INSTANCE.flickerOrder,
//                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closesortedLaunch,
                Index.INSTANCE.closeunsortedlaunch,
//                new IfElseCommand(
//                        () -> subsystems.hasAnyBalls,
//                        Index.INSTANCE.closesortedLaunch
//                ),
                Index.INSTANCE.closeunsortedlaunch,
//                Index.INSTANCE.secondcloseunsortedlaunch,
                subsystems.Turret.INSTANCE.home,
                new TurnTo(Angle.fromDeg(0))
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        subsystems.Turret.initPos = posConstants.redGoalInit;
        subsystems.Turret.INSTANCE.redgoalinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redgoalnine.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(122, 126, Math.toRadians(-143.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}