package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.bluefarline;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;
import org.firstinspires.ftc.teamcode.util.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "LINE blue Far-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Blue Far")
public class LINEM3BlueFar extends NextFTCOpMode {



    public LINEM3BlueFar() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE, Index.INSTANCE, Intake.INSTANCE, subsystems.Turret.INSTANCE, subsystems.Camera.INSTANCE, subsystems.ColorSensing.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
//                Thrower.INSTANCE.farshoot,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.1),
                Index.INSTANCE.farunsortedlaunch,
                Index.INSTANCE.farunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),
                Turret.INSTANCE.bluefar,
                new WaitUntil(() -> Math.abs(Math.abs(Turret.turret.getCurrentPosition()) - Math.abs(Turret.turretTargetPos)) <= 1),
                new FollowPath(bluefarline.line1, true),
                new Delay(0.25),
                new InstantCommand(() -> {Intake.negative = false;}),
                new FollowPath(bluefarline.launch1, true),
                new Delay(1), //0.75
                Index.INSTANCE.farunsortedlaunch,
                new InstantCommand(() -> {Intake.negative = true;}),
                new FollowPath(bluefarline.line2, false),
                new Delay(0.5),
                new FollowPath(bluefarline.launch2, true),
                new Delay(1), //0.5
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(bluefarline.offline, true)


//                new FollowPath(bluefar.line2, true),
//                new FollowPath(bluefar.launch2, true),
//                new FollowPath(bluefar.offline, true)


        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.blueFarInit;
        Turret.INSTANCE.bluefarinit.schedule();
//        Thrower.INSTANCE.farhood.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        bluefarline.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(60, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}