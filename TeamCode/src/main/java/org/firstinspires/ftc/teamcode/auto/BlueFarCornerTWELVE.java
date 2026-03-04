package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.BlueFarCorners12;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Camera;
import org.firstinspires.ftc.teamcode.util.subsystems.ColorSensing;
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
@Autonomous(name = "TWELVE CORNER blue Far-Meet 3", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "blue Far")
public class BlueFarCornerTWELVE extends NextFTCOpMode {



    public BlueFarCornerTWELVE() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE,
                        Index.INSTANCE,
                        Intake.INSTANCE,
                        Turret.INSTANCE,
                        Camera.INSTANCE,
                        ColorSensing.INSTANCE),
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
                Turret.INSTANCE.bluefar,
                new FollowPath(BlueFarCorners12.line1, true),
                new Delay(0.25),
                new FollowPath(BlueFarCorners12.launch1, true),
                Intake.INSTANCE.outtake,
                new Delay(1), //0.5
                new WaitUntil(() -> Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(BlueFarCorners12.corner1, false),
                new Delay(0.1),
                new FollowPath(BlueFarCorners12.bump1, false),
                new Delay(0.5),
                new FollowPath(BlueFarCorners12.launch2, true),
                new Delay(1), //0.5
                Intake.INSTANCE.outtake,
                new WaitUntil(() -> Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                Intake.INSTANCE.intake,
                new FollowPath(BlueFarCorners12.corner1, false),
                new Delay(0.1),
                new FollowPath(BlueFarCorners12.bump1, false),
                new Delay(0.5),
                new FollowPath(BlueFarCorners12.launch2, true),
                new Delay(1), //0.5
                Intake.INSTANCE.outtake,
                new WaitUntil(() -> Turret.INSTANCE.atposition),
                Index.INSTANCE.farunsortedlaunch,
                new FollowPath(BlueFarCorners12.offline, true)


//                new FollowPath(bluefar.line2, true),
//                new FollowPath(bluefar.launch2, true),
//                new FollowPath(bluefar.offline, true)


        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
        Turret.initPos = posConstants.blueFarInit;
        Turret.INSTANCE.bluefarinit.schedule();
        subsystems.far = true;
//        Thrower.INSTANCE.farhood.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        BlueFarCorners12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(60, 9, Math.toRadians(90)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}