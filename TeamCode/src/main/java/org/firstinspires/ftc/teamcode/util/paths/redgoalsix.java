package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class redgoalsix {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain offline;
    public static PathChain preload;
    public static PathChain gate;

    public static void BuildTrajectories(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(117.500, 131.500),

                                new Pose(87.000, 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(36.5))

                .build();

        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87.000, 110.000),
                                new Pose(89.117, 86.707),
                                new Pose(81.083, 85.072),
                                new Pose(122.000, 86.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))

                .build();

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(122.000, 86.000),
                                new Pose(107.243, 80.480),
                                new Pose(124.000, 77.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 77.500),

                                new Pose(87.000, 110.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.000, 110.000),

                                new Pose(105.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))

                .build();
    }
}
