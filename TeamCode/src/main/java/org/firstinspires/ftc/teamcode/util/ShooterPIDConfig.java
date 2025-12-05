package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class ShooterPIDConfig {



    // 6000 RPM goBILDA motors — they generally use 28 CPR encoders.
    // If you have no gearbox, then ticksPerRev = 28.
    // If you have a gearbox, do: 28 * gearRatio.
    public static double TICKS_PER_REV = 28.0;

    // These will be tunable in Dashboard
    public static double kP = 100;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.000357;

    // Desired shooter RPM (dashboard-adjustable)
    public static double targetRpm = 0;

    public static double turnPower = 0.3; // 0.35

    public static double hoodPos = 0.25;
    public static boolean autoAim = false;
}
