package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class goalred {
    public static PathChain preload;
    public static PathChain offline;
    public static PathChain line1;
    public static PathChain line1launch;
    public static PathChain autoreturn;


    public static void BuildTrajectories(Follower follower) {
        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120.000, 128.000), new Pose(85.000, 100.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(216))
                .build();

        offline = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 100.000),
                                new Pose(77.646, 81.667),
                                new Pose(100.000, 83.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(0))
                .build();

        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 83.500), new Pose(120.000, 83.368))
                )
                .setTangentHeadingInterpolation()
                .build();

        line1launch = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.000, 83.368),
                                new Pose(81.667, 82.595),
                                new Pose(101.929, 93.577),
                                new Pose(85.000, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .build();
    }
}
