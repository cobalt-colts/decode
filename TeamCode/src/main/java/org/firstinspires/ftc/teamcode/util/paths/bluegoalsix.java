package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class bluegoalsix {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain offline;
    public static PathChain preload;
    public static PathChain gate;

    public static void BuildTrajectories(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.500, 131.500),

                                new Pose(57.000, 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(143.5))

                .build();

        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.000, 110.000),
                                new Pose(54.726, 92.967),
                                new Pose(62.604, 90.080),
                                new Pose(22.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))

                .build();

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(22.000, 90.000),
                                new Pose(36.757, 80.480),
                                new Pose(20.000, 78.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 78.000),

                                new Pose(57.000, 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 110.000),

                                new Pose(39.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))

                .build();
    }
}
