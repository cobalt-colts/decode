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
    public static PathChain launch2;
    public static PathChain line2;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 123.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(100.000, 63.000),
                                new Pose(132.000, 63.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(91.000, 70.855),
                                new Pose(127.000, 70.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(127.000, 70.000),
                                new Pose(122.431, 62.277),
                                new Pose(130.000, 52.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        returnToLaunch = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 52.000),
                                new Pose(102.215, 59.031),
                                new Pose(91.000, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.000, 63.000),
                                new Pose(100.000, 63.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(100.000, 85.000),
                                new Pose(125.000, 85.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 85.000),
                                new Pose(81.143, 83.712),
                                new Pose(87.000, 110.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
