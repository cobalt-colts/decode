package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFarCorners12 {

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
                                new Pose(59.425, 9.208),
                                new Pose(59.312, 41.458),
                                new Pose(48.761, 34.833),
                                new Pose(11.000, 35.500)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.000, 35.500),
                                new Pose(54.000, 15.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        corner1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(36.000, 15.000),
                                new Pose(15.625, 44.253),
                                new Pose(11.370, 14.918)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();

        bump1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.370, 14.918),
                                new Pose(8.631, 8.783)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.631, 8.783),
                                new Pose(41.745, 8.386)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        corner2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(41.745, 8.386),
                                new Pose(41.275, 25.583),
                                new Pose(11.893, 14.414),
                                new Pose(12.246, 22.331),
                                new Pose(9.256, 31.443)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        bump2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.256, 31.443),
                                new Pose(12.839, 20.802),
                                new Pose(8.713, 13.635),
                                new Pose(8.670, 8.961)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.670, 8.961),
                                new Pose(41.889, 8.395)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        offline = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(41.889, 8.395),
                                new Pose(39.000, 33.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
    }
}
