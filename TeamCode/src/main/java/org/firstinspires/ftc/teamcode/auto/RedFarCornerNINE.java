package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.paths.RedFarCorner9;
import org.firstinspires.ftc.teamcode.util.positions;

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
@Autonomous(name = "RED Far Corner-STATE", preselectTeleOp = "STATE Teleop", group = "Red Far")
public class RedFarCornerNINE extends NextFTCOpMode {

    public RedFarCornerNINE() throws InterruptedException {
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
                // Shoot preloaded balls
                Turret.INSTANCE.redfar,
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Pick up line 1, drive to launch position, shoot
                Intake.INSTANCE.intake,
                Index.INSTANCE.transfer2,
                Turret.INSTANCE.redfar,
                new WaitUntil(() -> Math.abs(Turret.turret.getCurrentPosition() - Turret.turretTargetPos) <= 1),
                new FollowPath(RedFarCorner9.line2, false),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(RedFarCorner9.launch2, true),
                new Delay(1),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Pick up line 2, drive to launch position, shoot
                Intake.INSTANCE.intake,
                Index.INSTANCE.transfer2,
                new FollowPath(RedFarCorner9.line2, false),
                new Delay(1),
                Index.INSTANCE.down2,
                Intake.INSTANCE.outtake,
                new FollowPath(RedFarCorner9.launch2, true),
                new Delay(1),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),

                // Park
                new FollowPath(RedFarCorner9.offline, true)
        );
    }

    @Override public void onInit() {
        positions.redAlliance = true;
        subsystems.far = true;

        Index.INSTANCE.alldown.schedule();
//        Turret.initPos = posConstants.redFarInit;
//        Turret.INSTANCE.redfarinit.schedule();
        Turret.initPos = posConstants.redFarPickup;
        Turret.INSTANCE.redfar.schedule();
    }
    @Override public void onWaitForStart() {
        subsystems.Camera.INSTANCE.scanMotifSingle();
    }
    @Override
    public void onStartButtonPressed() {
        if (subsystems.motif == subsystems.motifs.NONE) {
            subsystems.motif = subsystems.motifs.GPP;
        }

        subsystems.start = true;
        RedFarCorner9.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(90, 15, Math.toRadians(0)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }
}