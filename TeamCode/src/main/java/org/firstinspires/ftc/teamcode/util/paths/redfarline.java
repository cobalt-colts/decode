package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class redfarline {

    public static PathChain launch1;
    public static PathChain line1;
    public static PathChain launch2;
    public static PathChain line2;
    public static PathChain offline;
    public static PathChain line2bump;

    public static void BuildTrajectories(Follower follower) {
        line1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 9.000),
                                new Pose(82.604, 41.250),
                                new Pose(95.230, 34.833),
                                new Pose(133.000, 35.500)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        launch1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 35.500),

                                new Pose(90.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))//.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90.000, 15.000),
                                new Pose(92.817, 73.878),
                                new Pose(117.509, 56.346),
                                new Pose(133.000, 58.000) //56
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        launch2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.000, 56.000), // 59.500

                                new Pose(90.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        offline = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.000, 15.000),

                                new Pose(105.000, 34.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }
}
