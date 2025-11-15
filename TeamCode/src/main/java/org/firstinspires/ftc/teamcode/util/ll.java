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
                flywheelspeed = 2300;
            }
        } else {
            flywheelspeed = 1600;
        }
        return flywheelspeed;
    }
    public static double fetchAlignment(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            double horizontalOffset = -result.getTy();
            double turnPower = 0.25;
            double tolerance = 1;

            if (Math.abs(horizontalOffset) > tolerance) {
//                if (horizontalOffset > 0) return 0.2;
//                else return -0.2;
                return  (horizontalOffset / 10);
            } else return 0;
        }
        else return 6767;
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
