package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12.*;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.util.paths.RedGoalLines12;
import org.firstinspires.ftc.teamcode.util.subsystems;


import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

public class redclose extends NextFTCOpMode {

    public Command autonomousRoutine() {
        return new SequentialGroupFixed(
                new FollowPath(launch1)
        );
    }
    public void AutonomousProgram() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {
        subsystems.start = true;
        RedGoalLines12.BuildTrajectories(PedroComponent.Companion.follower());
        PedroComponent.Companion.follower().setStartingPose(new Pose(120, 123, Math.toRadians(37)));

       autonomousRoutine().schedule();;
    }


}
