package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class bluefarline {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain offline;

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
                                new Pose(54.313, 58.070),
                                new Pose(39.169, 54.625),
                                new Pose(11.000, 58.000) // 55
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 55.000),

                                new Pose(54.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 15.000),

                                new Pose(39.000, 34.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }
}
