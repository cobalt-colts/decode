package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFarCorners12 {

    public static PathChain line1;
    public static PathChain launch1;
    public static PathChain corner1;
    public static PathChain bump1;
    public static PathChain launch2;
    public static PathChain corner2;
    public static PathChain bump2;
    public static PathChain launch3;
    public static PathChain offline;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.575, 9.208),
                                new Pose(84.688, 41.458),
                                new Pose(95.239, 35.667),
                                new Pose(133.000, 35.500)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 35.500),
                                new Pose(90.000, 15.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        corner1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.000, 15.000),
                                new Pose(108.000, 15.000),
                                new Pose(128.375, 44.253),
                                new Pose(132.630, 14.918)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        bump1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.630, 14.918),
                                new Pose(135.369, 8.783)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.369, 8.783),
                                new Pose(121.812, 17.169),
                                new Pose(102.255, 8.386)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        corner2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(102.255, 8.386),
                                new Pose(102.725, 25.583),
                                new Pose(132.107, 14.414),
                                new Pose(131.754, 22.331),
                                new Pose(134.744, 31.443)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .setReversed()
                .build();

        bump2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.744, 31.443),
                                new Pose(131.161, 20.802),
                                new Pose(135.287, 13.635),
                                new Pose(135.330, 8.961)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.330, 8.961),
                                new Pose(102.111, 8.395)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        offline = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(102.111, 8.395),
                                new Pose(84.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }
}
