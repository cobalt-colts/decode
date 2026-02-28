package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class ninebluefarcorner {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain offline;
    public static PathChain bump2;
    public static PathChain bump1;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 9.000),
                                new Pose(61.396, 41.250),
                                new Pose(48.761, 34.833),
                                new Pose(11.000, 35.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 35.500),

                                new Pose(54.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(44.887, 12.043, Math.toRadians(135)),
                                new Pose(26.991, 9.139),
                                new Pose(12.000, 8.000)
                        )
                )

                .build();

        bump1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 8.000),

                                new Pose(19.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        bump2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.000, 8.000),

                                new Pose(12.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 8.000),

                                new Pose(54.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 15.000),

                                new Pose(39.000, 33.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }
}
