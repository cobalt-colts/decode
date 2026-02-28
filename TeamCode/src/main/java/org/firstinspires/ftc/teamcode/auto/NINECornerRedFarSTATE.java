package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redfarcorner;
import org.firstinspires.ftc.teamcode.util.paths.redgoalnine;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

//@Disabled
@Config
@Autonomous(name = "state far corner red", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Goal")
public class NINECornerRedFarSTATE extends NextFTCOpMode {



    public NINECornerRedFarSTATE() throws InterruptedException {
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
//                subsystems.Thrower.INSTANCE.farshoot,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.25),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                subsystems.Turret.INSTANCE.redfar,
                new FollowPath(redfarcorner.line1, true),
                new Delay(0.25),
                new FollowPath(redfarcorner.launch1, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(redfarcorner.line2, false),
                new FollowPath(redfarcorner.bump1, false),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new FollowPath(redfarcorner.bump2, true)
                ),
                new Delay(0.5),
                new FollowPath(redfarcorner.launch2, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(redfarcorner.offline, true)


//                new FollowPath(redfar.line2, true),
//                new FollowPath(redfar.launch2, true),
//                new FollowPath(redfar.offline, true)


        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        subsystems.Turret.initPos = posConstants.redFarInit;
        subsystems.Turret.INSTANCE.redgoalinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redgoalnine.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(117.5, 131.5, Math.toRadians(-143.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}