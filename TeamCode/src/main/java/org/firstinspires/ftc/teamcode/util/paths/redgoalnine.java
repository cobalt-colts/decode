package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class redgoalnine {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain preload;
    public static PathChain gate;

    public static void BuildTrajectories(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(117.500, 131.500),

                                new Pose(85.000, 110.000) //87
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(36.5))

                .build();

        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 110.000),
                                new Pose(89.117, 86.707),
                                new Pose(81.083, 85.072),
                                new Pose(122.000, 88.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))

                .build();

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(122.000, 88.500),
                                new Pose(110.217, 82.985),
                                new Pose(124.000, 78.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 78.000),

                                new Pose(85.000, 110.000) //87
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 110.000),
                                new Pose(88.883, 79.993),
                                new Pose(85.304, 62.452),
                                new Pose(122.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(0))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.000, 63.000),

                                new Pose(85.000, 114.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))

                .build();
    }
}
