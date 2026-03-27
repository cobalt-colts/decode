package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFarCorner9 {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain bump1;
    public static PathChain bump2;
    public static PathChain offline;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 9.000),
                                new Pose(82.604, 41.250),
                                new Pose(95.239, 34.833),
                                new Pose(126, 37.5),
                                new Pose(133.000, 39.000) //37.5          35.5
                        )
                ).setTangentHeadingInterpolation()

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 35.500),

                                new Pose(90.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 15.000),
                                new Pose(45.887, 12.043),
                                new Pose(27.991, 9.139),
                                new Pose(12, 11.000) //8.000
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        bump1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 8.000),

                                new Pose(19.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

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
