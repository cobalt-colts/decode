package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedGoalLines12 {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain preload;
    public static PathChain gate;
    public static PathChain line3;
    public static PathChain launch3;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 123.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .addPath(
                        new BezierLine(
                                new Pose(126.000, 82.000),
                                new Pose(91.000, 92.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 92.000),
                                new Pose(102.068, 81.989),
                                new Pose(126.000, 82.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
