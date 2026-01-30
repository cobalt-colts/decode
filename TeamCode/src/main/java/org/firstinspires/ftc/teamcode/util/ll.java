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
        double flywheelspeed = 0;

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
            if (ta >= 0.5) {
                // flywheelspeed = 2161 * Math.pow(0.86, ta); // For Ri3D bot
                flywheelspeed = 1650 * Math.pow(0.90446, ta); //1983.72
            } else {
                flywheelspeed = farSpeed + positions.flyWheelCorrect; // 2150
            }
        } else {
            flywheelspeed = farSpeed + positions.flyWheelCorrect; // 2150
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
