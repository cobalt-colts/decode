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
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;

@Config
@Autonomous(name = "LINE BLUE Far-STATE", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Blue Far")
public class NINELineBlueFarSTATE extends NextFTCOpMode {

    public NINELineBlueFarSTATE() throws InterruptedException {
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
                Turret.INSTANCE.bluefar,
                new WaitUntil(() -> Math.abs(Turret.turret.getCurrentPosition() - Turret.turretTargetPos) <= 1),
                new FollowPath(bluefarline.line1, true),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(bluefarline.launch1, true),
                new Delay(0.75),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Pick up line 2, drive to launch position, shoot
                Intake.INSTANCE.intake,
                Index.INSTANCE.transfer2,
                new FollowPath(bluefarline.line2, false),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(bluefarline.launch2, true),
                new Delay(1),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Park
                new FollowPath(bluefarline.offline, true)
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.blueFarInit;
        Turret.INSTANCE.bluefarinit.schedule();
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