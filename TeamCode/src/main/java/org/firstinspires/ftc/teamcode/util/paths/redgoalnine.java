package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class redgoalnine {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain preload;
    public static PathChain gate;
    public static PathChain line3;
    public static PathChain launch3;

    public static void BuildTrajectories(Follower follower) {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(122.000, 126.000),
                                new Pose(89.378, 110.565),
                                new Pose(85.000, 92.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143.5), Math.toRadians(0))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 92.000),
                                new Pose(90.313, 89.339),
                                new Pose(120.000, 92.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.000, 92.000),
                                new Pose(106.470, 83.320),
                                new Pose(122.000, 82.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.000, 82.000),
                                new Pose(85.000, 90.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(94.326, 58.372),
                                new Pose(101.491, 65.693),
                                new Pose(122.000, 64.500) //120
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.000, 64.500),
                                new Pose(97.104, 67.724),
                                new Pose(85.000, 90.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(90.430, 31.433),
                                new Pose(97.352, 38.763),
                                new Pose(122.000, 40.500) //120
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 40.500),
                                new Pose(92.000, 110.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }
}
