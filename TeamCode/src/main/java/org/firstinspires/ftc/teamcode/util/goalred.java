package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class goalred {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain offline;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.501, 124.975), new Pose(91.000, 96.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(220))
                .build();

        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.000, 96.000), new Pose(91.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.000, 87.500), new Pose(113.500, 87.500))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        launch2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(113.500, 87.500), new Pose(91.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .build();

        offline = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.000, 96.000), new Pose(91.000, 110.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(220))
                .build();
    }
}
