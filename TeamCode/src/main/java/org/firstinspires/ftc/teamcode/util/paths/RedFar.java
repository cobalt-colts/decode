package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFar {

    public static PathChain launch1;
    public static PathChain farline;
    public static PathChain cycle;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.500, 9.500),
                                new Pose(82.500, 15.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        cycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(82.500, 15.000),
                                new Pose(93.768, 29.936),
                                new Pose(99.063, 9.335),
                                new Pose(131.572, 10.145)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(131.572, 10.145),
                                new Pose(120.000, 11.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 11.000),
                                new Pose(131.572, 10.145)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(131.572, 10.145),
                                new Pose(82.500, 15.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        farline = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(82.500, 15.000),
                                new Pose(92.000, 37.000),
                                new Pose(127.000, 34.750)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(127.000, 34.750),
                                new Pose(92.000, 37.000),
                                new Pose(82.500, 15.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
