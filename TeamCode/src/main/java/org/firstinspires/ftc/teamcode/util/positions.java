package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.posConstants.closeHood;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker2down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker3down;
import static org.firstinspires.ftc.teamcode.util.posConstants.pinpoint;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;

//import org.firstinspires.ftc.teamcode.Prism.Color;

@Config
//@Configurable
@SuppressWarnings("CannotResolve")
public class positions {
    public static boolean redAlliance = true;

    public static double intakePower;

    public static char ball1 = 'b';
    public static char ball2 = 'b';
    public static char ball3 = 'b';
    public static int greenPos = 3;
    public static double flicker1Pos = flicker1down;
    public static double flicker2Pos = flicker2down;
    public static double flicker3Pos = flicker3down;
    public static boolean allDown;
    public enum Index { HOLD, DOWN, UP, AUTODOWN, AUTOUP}
    public static Index index = Index.HOLD;
    public static Index flick3 = Index.HOLD;
    public static Index flick2 = Index.HOLD;
    public static Index flick1 = Index.HOLD;

    public static double targetTps = 0;
    public static int targetVelocity = 0;
    public static double power = 0;
    public static double currentVelocity = 0;
    public static double hoodPos = 0.05; // bottom is 0.35, top is 0.
    public static boolean autoTurret = true;
    public static double turretPos = 0;
    public static boolean canShoot;
    public static int flyWheelCorrect = 0;

    public static double frontLeftPower;
    public static double frontRightPower;
    public static double backLeftPower;
    public static double backRightPower;

    public static boolean startReady;

    public static void reset() {
        redAlliance = true;
        intakePower = 0;
        flicker1Pos = flicker1down;
        flicker2Pos = flicker2down;
        flicker3Pos = flicker3down;
        targetTps = 0;
        hoodPos = closeHood;
        autoTurret = false;
        canShoot = false;
        pinpoint.resetPosAndIMU();
    }
}
