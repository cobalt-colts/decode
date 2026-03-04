package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class bluegoalnine {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain launch3;
    public static PathChain line3;
    public static PathChain preload;
    public static PathChain gate;

    public static void BuildTrajectories(Follower follower) {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(22.000, 126.000),
                                new Pose(54.622, 110.565),
                                new Pose(53.522, 81.426)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(323.5), Math.toRadians(180))
                .build();

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.522, 81.426),
                                new Pose(53.687, 81.991),
                                new Pose(24.783, 83.617)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.783, 83.617),
                                new Pose(37.530, 70.320),
                                new Pose(11.043, 68.417)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.043, 68.417),
                                new Pose(53.522, 78.896) //81.896
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.522, 78.896), //81.896
                                new Pose(49.674, 58.372),
                                new Pose(42.509, 55.693),
                                new Pose(24.313, 52.483) //54.483
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.313, 54.483),
                                new Pose(46.896, 67.724),
                                new Pose(53.522, 78.896) //81.896
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.522, 78.896), //81.896
                                new Pose(53.570, 31.433),
                                new Pose(46.648, 38.763),
                                new Pose(23.374, 33.961) //35.961
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.374, 33.961),
                                new Pose(52.000, 110.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
}
