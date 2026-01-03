package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class redfar {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain offline;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.000, 9.000),
                                new Pose(82.761, 30.607),
                                new Pose(125.000, 35.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.000, 35.500),
                                new Pose(89.000, 43.000),
                                new Pose(89.000, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.000, 92.000),
                                new Pose(88.374, 79.563),
                                new Pose(125.000, 83.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 83.500),

                                new Pose(89.000, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 92.000),

                                new Pose(89.000, 115.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();
    }
}
