package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;

import java.util.List;

import dev.nextftc.ftc.ActiveOpMode;


public class ll {
    public static double fetchFlywheelSpeed(Limelight3A limelight) {

        double ta = 0;
        double flywheelspeed = Double.NaN; // Assume the worst

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Make sure the AprilTag is a TARGET AprilTag (and not a motif AprilTag):
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 24) {
                    ta = result.getTa();

                    ActiveOpMode.telemetry().addData("ta: ", ta);
                    ActiveOpMode.telemetry().addData("fiducial ID", id);

                            // Removed /2 This returns speed in rpm. Convert to ticks per second outside this code
                    flywheelspeed = ((1752.97 - (211.567 * ta))); // 323.74855 * Math.pow(0.996013, ta);
                    flywheelspeed *= .89;
                    ActiveOpMode.telemetry().addData("flywheelspeed (in ll): ", flywheelspeed);
                }
            }
        }
        return flywheelspeed;
    }
    public static double fetchAlignment(Limelight3A limelight, boolean redAlliance) {
        final int RED_GOAL_TAG = 24;
        final int BLUE_GOAL_TAG = 20;
        int targetTag = redAlliance ? RED_GOAL_TAG : BLUE_GOAL_TAG;
        int far = redAlliance ? -1 : 1;
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : fiducials) {
                if (tag.getFiducialId() == targetTag) {
                    double horizontalOffset = result.getTy(); //angle
                    if (result.getTa() < 0.5) horizontalOffset += far * posConstants.farAngleOffset;
                    if (Math.abs(horizontalOffset) > tolerance) {
                        return (-horizontalOffset * ticksPerDegree);
                    } else return 0;
                }
            }
        }
        return Double.NaN;
    }
    public static double fetchHoodPos(Limelight3A limelight) {
        double ta = 0;
        double hoodpos = .3;
        limelight.setPollRateHz(100);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
            // flywheelspeed = 2161 * Math.pow(0.86, ta); // For Ri3D bot
            hoodpos = ((.1436 * ta) /* + 0.05777 */ ); // 0.30707 * Math.pow(121.29199, ta);
            hoodpos = Math.max(0.13, hoodpos);
            hoodpos = Math.min(0.7, hoodpos);
            hoodpos *= 2;
        } else {
//            hoodpos = .2;
            hoodpos = Double.NaN;
        }
        return hoodpos;
    }
}
