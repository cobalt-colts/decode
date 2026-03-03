package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1up;
import static org.firstinspires.ftc.teamcode.util.subsystems.isoccupied;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.paths.redgoalnine;
import org.firstinspires.ftc.teamcode.util.posConstants;
import org.firstinspires.ftc.teamcode.util.subsystems;
import org.firstinspires.ftc.teamcode.util.subsystems.Index;
import org.firstinspires.ftc.teamcode.util.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.subsystems.Thrower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.positionable.SetPosition;

import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;

@Config
@Autonomous(name = "NextFTCTestingGC", preselectTeleOp = "Sriram's ChatGPT TeleOp", group = "Red Goal")
public class NextFTCTestingGC extends NextFTCOpMode {



    public NextFTCTestingGC() throws InterruptedException {
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
                subsystems.Camera.INSTANCE.setmotif,

                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                Index.INSTANCE.closeSortedLaunch(), // Launch in motif order
                Index.INSTANCE.sensedunsorted  // In case one didn't launch, order doesn't matter just get it launched
        );
    }

    @Override public void onInit() {
        Index.INSTANCE.alldown.schedule();
//        subsystems.Turret.initPos = posConstants.redGoalInit;
//        subsystems.Turret.INSTANCE.redgoalinit.schedule();
    }
    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        redgoalnine.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(122, 126, Math.toRadians(-143.5)));
        autoRoutine().schedule();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() {
        subsystems.start = false;
    }

}