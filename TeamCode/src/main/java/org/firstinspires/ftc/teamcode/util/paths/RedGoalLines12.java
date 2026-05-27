package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedGoalLines12 {

    public static PathChain launch1;
    public static PathChain line1;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 123.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(50))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(100.000, 65.000),
                                new Pose(123.000, 65.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(123.000, 65.000),
                                new Pose(100.000, 65.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
