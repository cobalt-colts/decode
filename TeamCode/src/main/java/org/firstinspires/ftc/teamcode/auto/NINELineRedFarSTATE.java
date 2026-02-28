package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redfarline;
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
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "LINE RED Far-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Far")
public class NINELineRedFarSTATE extends NextFTCOpMode {



    public NINELineRedFarSTATE() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE,
                        Index.INSTANCE,
                        Intake.INSTANCE,
                        subsystems.Turret.INSTANCE,
                        subsystems.Camera.INSTANCE,
                        subsystems.ColorSensing.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {
        subsystems.far = true;

        return new SequentialGroup(
//                Thrower.INSTANCE.farshoot,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.1),
                Index.INSTANCE.farunsortedlaunch,
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                Turret.INSTANCE.redfar,
                new WaitUntil(() -> Math.abs(Math.abs(Turret.turret.getCurrentPosition()) - Math.abs(Turret.turretTargetPos)) <= 1),
                new FollowPath(redfarline.line1, true),
                new Delay(1), //0.5
                Intake.INSTANCE.intake,
                new FollowPath(redfarline.launch1, true),
                new Delay(0.75),
                Index.INSTANCE.farunsortedlaunch,
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(redfarline.line2, false),
                new Delay(1), //0.5
                new FollowPath(redfarline.launch2, true),
                new Delay(1), //0.5
                Index.INSTANCE.farunsortedlaunch,
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(redfarline.offline, true)


//                new FollowPath(redfar.line2, true),
//                new FollowPath(redfar.launch2, true),
//                new FollowPath(redfar.offline, true)


        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.redFarInit;
        Turret.INSTANCE.redfarinit.schedule();
//        Thrower.INSTANCE.farhood.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redfarline.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(84, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}