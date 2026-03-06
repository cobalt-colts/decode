package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.positions;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.RedFarCorner9;
import org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

//@Disabled
@Config
@Autonomous(name = "RED Far Corner-STATE", preselectTeleOp = "STATE Teleop", group = "Red Goal")
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

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroupFixed(
//                subsystems.Thrower.INSTANCE.farshoot,

                Intake.INSTANCE.intake,
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new Delay(0.25),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),
                subsystems.Turret.INSTANCE.redfar,
                new FollowPath(RedFarCorner9.line1, true),
                new Delay(0.25),
//                Intake.INSTANCE.outtake,
                new FollowPath(RedFarCorner9.launch1, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Intake.INSTANCE.intake,
                new FollowPath(RedFarCorner9.line2, false),
                new FollowPath(RedFarCorner9.bump1, false),
                new ParallelDeadlineGroup(
                        new Delay(2),
                        new FollowPath(RedFarCorner9.bump2, true)
                ),
                new Delay(0.5),
//                Intake.INSTANCE.outtake,
                new FollowPath(RedFarCorner9.launch2, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
                Index.INSTANCE.farSortedLaunch(),
                Index.INSTANCE.sensedunsortedatspeed(),
                Index.INSTANCE.sensedunsortedatspeed(),
                new FollowPath(RedFarCorner9.offline, true)


//                new FollowPath(redfar.line2, true),
//                new FollowPath(redfar.launch2, true),
//                new FollowPath(redfar.offline, true)


        );
    }

    @Override public void onInit() {
        positions.redAlliance = true;
        subsystems.far = true;

        Index.INSTANCE.alldown.schedule();
        subsystems.Turret.initPos = posConstants.redFarInit;
        subsystems.Turret.INSTANCE.redfarinit.schedule();
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
        PedroComponent.Companion.follower().setStartingPose(new Pose(84.0, 9.0, Math.toRadians(-270)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}