package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ParentHub;

public class blueGoalLines12 {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain gate;
    public static PathChain returnToLaunch;
    public static PathChain launch2;
    public static PathChain line2;
//    public static PathChain gate2;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.500, 123.000),
                                new Pose(50.500, 92.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(143))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.500, 92.000),
                                new Pose(41.500, 58.000),
                                new Pose(7.000, 58.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.500, 92.000),
                                new Pose(50.247, 66.073),
                                new Pose(14.374, 71.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(14.374, 71.000),
                                new Pose(21.091, 57.728),
                                new Pose(11.500, 52.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        returnToLaunch = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.500, 52.000),
                                new Pose(39.285, 59.031),
                                new Pose(50.500, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(7.000, 58.000),
                                new Pose(41.500, 59.462),
                                new Pose(50.500, 92.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.500, 92.000),
                                new Pose(41.500, 85.000),
                                new Pose(16.500, 85.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(16.500, 85.000),
                                new Pose(48.196, 85.000),
                                new Pose(60.000, 115.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
