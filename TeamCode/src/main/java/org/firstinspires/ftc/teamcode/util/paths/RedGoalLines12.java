package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedGoalLines12 {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain gate;
    public static PathChain returnToLaunch;

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
                                new Pose(100.000, 66.000),
                                new Pose(123.000, 66.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(123.000, 66.000),
                                new Pose(100.000, 66.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(102.776, 71.920),
                                new Pose(125.000, 72.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 72.000),
                                new Pose(122.431, 62.277),
                                new Pose(132.000, 56.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        returnToLaunch = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.000, 56.000),
                                new Pose(102.215, 59.031),
                                new Pose(91.000, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
