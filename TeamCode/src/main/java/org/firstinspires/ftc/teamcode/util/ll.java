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
                if (id == 24 || id == 20) {
                    ta = result.getTa();

                    ActiveOpMode.telemetry().addData("ta: ", ta);
                    ActiveOpMode.telemetry().addData("fiducial ID", id);

                            // Removed /2 This returns speed in rpm. Convert to ticks per second outside this code
                    if (ta > 0.4) flywheelspeed = ((1752.97 - (211.567 * ta))); // 323.74855 * Math.pow(0.996013, ta);
                    else if (ta < 0.1) flywheelspeed = Double.NaN;
                    else flywheelspeed = 1950; // 2000 constant far zone speed
                    flywheelspeed *= .89;
                    ActiveOpMode.telemetry().addData("flywheelspeed (in ll): ", flywheelspeed);
                }
            }
        }
        else flywheelspeed = Double.NaN;
        return flywheelspeed;
    }
    public static double fetchAlignment(Limelight3A limelight) {
        if (limelight == null) {
            return Double.NaN;
        }

        limelight.start();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return Double.NaN;
        }

        double tx = Double.NaN;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 24 || id == 20) {
                    tx = fiducial.getTargetXDegrees();
                    break;
                }
            }
        }

        if (Double.isNaN(tx)) {
            tx = result.getTx();
        }

        if (Double.isNaN(tx) || Math.abs(tx) <= tolerance) {
            return 0;
        }

        double correction = (0.00002304 * tx * Math.abs(tx)) + (1.2 * tx);
        return correction;
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
            if (ta > 0.4) hoodpos = ((.13 + .1436 * ta) /* + 0.05777 */ ); // 0.30707 * Math.pow(121.29199, ta);   // xxx fudge factor // miles 2/23/26
            else hoodpos = (farHood);
            hoodpos = Math.max(0.08, hoodpos); //.13
            hoodpos = Math.min(0.7, hoodpos);
        } else {
//            hoodpos = .2;
            hoodpos = Double.NaN;
        }
        return hoodpos;
    }
}
