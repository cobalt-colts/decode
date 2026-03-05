package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.BlueGoalLines12;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous(name = "CAMERA TEST", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "blue Goal")
public class CAMERATEST extends NextFTCOpMode {



    public CAMERATEST() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE,
                        Index.INSTANCE,
                        Intake.INSTANCE,
//                        subsystems.Turret.INSTANCE,
                        subsystems.Camera.INSTANCE,
                        subsystems.ColorSensing.INSTANCE),
                new PedroComponent(Constants::createFollower)

//                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {

        return new SequentialGroup(
//                subsystems.Turret.INSTANCE.home,
                Thrower.INSTANCE.shooteron,
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                new InstantCommand(() -> ActiveOpMode.telemetry().addLine("launching")),
//                Index.INSTANCE.colorsensecloseunsortedlaunch,
                new InstantCommand(() -> ActiveOpMode.telemetry().addData("start: ", subsystems.start)),
                new InstantCommand(() -> ActiveOpMode.telemetry().addLine("ALL DONE")),
                new InstantCommand(() -> ActiveOpMode.telemetry().update())

//                subsystems.Camera.INSTANCE.setmotif,
////                subsystems.Turret.INSTANCE.bluegoal, miles, 1/16/25 (worked just fine w deafult paths, turret caused it to shoot outside field.)
//                new ParallelGroup(
//                        Thrower.INSTANCE.shooteron,
//                        new FollowPath(BlueGoalLine.preload, true),
//                        subsystems.Turret.INSTANCE.bluegoal
//                ),
//                new InstantCommand(() -> {Thrower.targetvelocity = 1370;}),
//                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
//                new WaitUntil(() -> subsystems.Turret.INSTANCE.atposition),
//                new Delay(0.25),
//
//                Index.INSTANCE.closeunsortedlaunch(),
//                Index.INSTANCE.closeunsortedlaunch(),
////                new IfElseCommand(
////                        () -> subsystems.hasAnyBalls,
////                        Index.INSTANCE.closesortedLaunch
////                ),
////                Index.INSTANCE.closeunsortedlaunch,
////                Index.INSTANCE.secondcloseunsortedlaunch,
//                Intake.INSTANCE.intake,
//
//                new FollowPath(BlueGoalLine.line1, true),
////                subsystems.Turret.INSTANCE.bluegoal,
//                new Delay(0.25),
////                new InstantCommand(() -> {Intake.outtake();}),
//                new FollowPath(BlueGoalLine.gate, true),
////                new Delay(0.5),
//                new FollowPath(BlueGoalLine.launch1, true),
//                new Delay(1), //0.5
////                Intake.INSTANCE.outtake,
////                Index.INSTANCE.flickerOrder,
////                Index.INSTANCE.closesortedLaunch,
////                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closeunsortedlaunch(),
//                Index.INSTANCE.closeunsortedlaunch(),
////                new IfElseCommand(
////                        () -> subsystems.hasAnyBalls,
////                        Index.INSTANCE.closesortedLaunch
////                ),
////                Index.INSTANCE.closeunsortedlaunch,
////                Index.INSTANCE.secondcloseunsortedlaunch,
//                Intake.INSTANCE.intake,
//                new FollowPath(BlueGoalLine.line2, true),
//                new Delay(0.5),
////                Intake.INSTANCE.outtake,
////                new InstantCommand(() -> {Intake.outtake();}),
//                new FollowPath(BlueGoalLine.launch2, true),
//                new Delay(1),
////                Index.INSTANCE.flickerOrder,
////                Index.INSTANCE.closesortedLaunch,
////                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closeunsortedlaunch(),
//                Index.INSTANCE.closeunsortedlaunch(),
////                new IfElseCommand(
////                        () -> subsystems.hasAnyBalls,
////                        Index.INSTANCE.closesortedLaunch
////                ),
////                Index.INSTANCE.secondcloseunsortedlaunch,
//                new InstantCommand(subsystems.Turret.INSTANCE.bluepark),
//                Intake.INSTANCE.intake,
//                new FollowPath(BlueGoalLine.line3, true),
//                new Delay(0.5),
////                Intake.INSTANCE.outtake,
////                new InstantCommand(() -> {Intake.outtake();}),
//                new InstantCommand(() -> {Thrower.targetvelocity = 1240;}),
//                new FollowPath(BlueGoalLine.launch3),
//                new Delay(0.5),
////                Index.INSTANCE.flickerOrder,
////                Index.INSTANCE.closesortedLaunch,
////                Index.INSTANCE.closesortedLaunch,
//                Index.INSTANCE.closeunsortedlaunch(),
//                Index.INSTANCE.closeunsortedlaunch(),
//
////                new IfElseCommand(
////                        () -> subsystems.hasAnyBalls,
////                        Index.INSTANCE.closesortedLaunch
////                ),
//                Index.INSTANCE.closeunsortedlaunch(),
////                Index.INSTANCE.secondcloseunsortedlaunch,
//                subsystems.Turret.INSTANCE.home,
//                new TurnTo(Angle.fromDeg(0))
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
//        subsystems.Turret.initPos = posConstants.blueGoalInit;
//        subsystems.Turret.INSTANCE.bluegoalinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        BlueGoalLines12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(22, 126, Math.toRadians(324)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}