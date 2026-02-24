package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class redfarcorner {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch;
    public static PathChain line2;
    public static PathChain offline;
    public static PathChain bump1;
    public static PathChain bump2;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 9.000),
                                new Pose(82.604, 41.250),
                                new Pose(95.239, 34.833),
                                new Pose(133.000, 35.500)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 35.500),

                                new Pose(90.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90.000, 15.000),
                                new Pose(99.113, 12.043),
                                new Pose(117.009, 9.139),
                                new Pose(132.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        bump1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.000, 8.000),

                                new Pose(125.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        bump2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 8.000),

                                new Pose(132.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.000, 8.000),

                                new Pose(90.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.000, 15.000),

                                new Pose(105.000, 33.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();
    }
}
