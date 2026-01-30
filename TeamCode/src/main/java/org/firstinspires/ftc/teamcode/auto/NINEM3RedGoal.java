package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redgoalnine;
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
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "(NINE) RED Close-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Goal")
public class NINEM3RedGoal extends NextFTCOpMode {



    public NINEM3RedGoal() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE, Index.INSTANCE, Intake.INSTANCE, subsystems.Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
//                subsystems.Turret.INSTANCE.redgoal, miles, 1/16/25 (worked just fine w deafult paths, turret caused it to shoot outside field.)
                new ParallelGroup(
                        Thrower.INSTANCE.goalshoot,
                        new FollowPath(redgoalnine.preload, true)
                ),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),

                new FollowPath(redgoalnine.line1, true),
//                subsystems.Turret.INSTANCE.redgoal,
                new Delay(0.25),
                new InstantCommand(() -> {Intake.negative = false;}),
                new FollowPath(redgoalnine.gate, true),
                new Delay(0.5),
                new FollowPath(redgoalnine.launch1, true),
                new Delay(0.5), //0.5
                Index.INSTANCE.closeunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),
                new FollowPath(redgoalnine.line2, true),
                new Delay(0.5),
                new InstantCommand(() -> {Intake.negative = false;}),
                new FollowPath(redgoalnine.launch2, true),
                new Delay(0.5),
                Index.INSTANCE.closeunsortedlaunch,
                new TurnTo(Angle.fromDeg(0))
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Thrower.INSTANCE.closehood.schedule();
        subsystems.Turret.turretTargetPos = 0;
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redgoalnine.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(117.5, 131.5, Math.toRadians(36.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}