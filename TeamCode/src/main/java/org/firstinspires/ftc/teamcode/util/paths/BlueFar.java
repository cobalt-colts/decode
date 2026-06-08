package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFar {

    public static PathChain launch1;
    public static PathChain farline;
    public static PathChain cycle;
    public static PathChain leave;

    public static void BuildTrajectories(Follower follower) {
        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(59.000, 9.500),
                                new Pose(59.000, 15.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        cycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 15.000),
                                new Pose(47.732, 29.936),
                                new Pose(42.437, 9.335),
                                new Pose(9.928, 10.145)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(9.928, 10.145),
                                new Pose(21.500, 11.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(21.500, 11.000),
                                new Pose(17, 10.145)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(9.928, 10.145),
                                new Pose(54.500, 15.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        farline = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.500, 15.000),
                                new Pose(49.500, 37.000),
                                new Pose(14.500, 34.750)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(14.500, 34.750),
                                new Pose(49.500, 37.000),
                                new Pose(54.500, 15.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.500, 15.000),
                                new Pose(38.000, 32.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }
}
