package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.RedFarLines9;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;
import org.firstinspires.ftc.teamcode.util.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;

@Config
@Autonomous(name = "LINE RED Far-STATE", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Far")
public class RedFarLineNINE extends NextFTCOpMode {

    public RedFarLineNINE() throws InterruptedException {
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
        subsystems.far = true;

        return new SequentialGroupFixed(
                // Shoot preloaded balls
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Pick up line 1, drive to launch position, shoot
                Intake.INSTANCE.intake,
                Index.INSTANCE.transfer2,
                Turret.INSTANCE.redfar,
                new WaitUntil(() -> Math.abs(Turret.turret.getCurrentPosition() - Turret.turretTargetPos) <= 1),
                new FollowPath(RedFarLines9.line1, true),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(RedFarLines9.launch1, true),
                new Delay(0.75),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Pick up line 2, drive to launch position, shoot
                Intake.INSTANCE.intake,
                Index.INSTANCE.transfer2,
                new FollowPath(RedFarLines9.line2, false),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(RedFarLines9.launch2, true),
                new Delay(1),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Park
                new FollowPath(RedFarLines9.offline, true)
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.redFarInit;
        Turret.INSTANCE.redfarinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        RedFarLines9.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(84, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }
}