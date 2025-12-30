package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.posConstants.lift1Down;
import static org.firstinspires.ftc.teamcode.util.posConstants.lift2Down;
import static org.firstinspires.ftc.teamcode.util.posConstants.lift3Down;

public class positions {
    public static boolean redAlliance = true;

    public static double intakePower;
    public enum Intake {IN, AUTO, OUT, HOLD}
    public static Intake intakeState = Intake.HOLD;

    public static char ball1 = 'b';
    public static char ball2 = 'b';
    public static char ball3 = 'b';
    public static char ballWant = '0';
    public static String indexOrder = "";
    public static int liftTime = 0;
    public static double lift1Pos = lift1Down;
    public static double lift2Pos = lift2Down;
    public static double lift3Pos = lift3Down;
    public static double lift1Posi;
    public static double lift2Posi;
    public static double lift3Posi;
    public enum Index {G, P, HOLD, U1, D1, U2, D2, U3, D3, DOWN}
    public static Index index = Index.HOLD;

    public static double targetTps = 0;
    public static double hoodPos = 0.05; // bottom is 0.35, top is 0.
    public static boolean autoTurret = false;
    public static double turretPos = 0;
    public static boolean canShoot;

    public static double frontLeftPower;
    public static double frontRightPower;
    public static double backLeftPower;
    public static double backRightPower;
}
