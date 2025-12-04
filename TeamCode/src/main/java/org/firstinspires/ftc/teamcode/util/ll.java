package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import java.util.List;


public class ll {
    public static double fetchFlywheelSpeed(Limelight3A limelight) {

        double ta = 0;
        double flywheelspeed = 0;

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
            if (ta >= 0.5) {
                // flywheelspeed = 2161 * Math.pow(0.86, ta); // For Ri3D bot
                flywheelspeed = 1943.72 * Math.pow(0.90446, ta);
            } else {
                flywheelspeed = 2200;
            }
        } else {
            flywheelspeed = 1600;
        }
        return flywheelspeed;
    }
    public static double fetchAlignment(Limelight3A limelight, boolean redAlliance) {
//        LLResult result = limelight.getLatestResult();
//        if(result != null && result.isValid()) {
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            double horizontalOffset = -result.getTy();
//            double turnPower = 0.25;
//            double tolerance = 1;
//
//            if (Math.abs(horizontalOffset) > tolerance) {
//                if (horizontalOffset > 0) return turnPower;
//                else return (turnPower * -1);
////                return  (horizontalOffset / 10);
//            } else return 0;
//        }
//        else return 6767;


        final int RED_GOAL_TAG = 24;
        final int BLUE_GOAL_TAG = 20;
        int targetTag = redAlliance ? RED_GOAL_TAG : BLUE_GOAL_TAG;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 6767;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        // --- Find the correct tag ---
        for (LLResultTypes.FiducialResult tag : fiducials) {

            if (tag.getFiducialId() == targetTag) {

                double horizontalOffset = -tag.getTargetYPixels();
                double tolerance = 5;
                double turnPower = 0.025 * horizontalOffset;

                if (Math.abs(horizontalOffset) > tolerance) {
                    return (horizontalOffset > 0) ? turnPower : -turnPower;
                } else {
                    return 0;  // centered
                }
            }
        }

        // If we reach here, no target tag seen
        return 6767;
    }
    public static double fetchHoodPos(Limelight3A limelight) {
        double ta = 0;
        double hoodpos = 0.5;
        limelight.setPollRateHz(100);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
            // flywheelspeed = 2161 * Math.pow(0.86, ta); // For Ri3D bot
            hoodpos = .33649 * Math.pow(1.2303, ta);
        } else {
            hoodpos = 0.5;
        }
        return hoodpos;
    }
}
