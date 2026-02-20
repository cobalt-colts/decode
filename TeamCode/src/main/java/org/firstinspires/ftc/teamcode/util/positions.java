package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.posConstants.closeHood;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker2down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker3down;
import static org.firstinspires.ftc.teamcode.util.posConstants.pinpoint;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

//import org.firstinspires.ftc.teamcode.Prism.Color;

@Config
@Configurable
@SuppressWarnings("CannotResolve")
public class positions {
    public static boolean redAlliance = true;

    public static double intakePower;
    public enum Intake {IN, AUTO, OUT, HOLD}
    public static Intake intakeState = Intake.HOLD;

    public static char rightBall = 'b';
    public static char backBall = 'b';
    public static char leftBall = 'b';
    public static char[] ballWant = new char[2];
    public static char[] balls = {'b', 'b', 'b'};
    public static String corralOrder = "PPP";
    public static String indexOrder = "";
    public static int liftTime = 0;
    public static double rightFlickerPos = flicker1down;
    public static double backFlickerPos = flicker2down;
    public static double leftFlickerPos = flicker3down;
    public static double rightFlickerPosi;
    public static double backFlickerPosi;
    public static double leftFlickerPosi;
    public static boolean allDown;
    public enum Index {G, P, HOLD, U1, D1, U2, D2, U3, D3, DOWN, UP}
    public static Index index = Index.HOLD;
    public static Index left = Index.DOWN;
    public static Index back = Index.DOWN;
    public static Index right = Index.DOWN;
    //    public static Color leftColor = Color.WHITE;
//    public static Color backColor = Color.WHITE;
//    public static Color rightColor = Color.WHITE;
    public static int ballNum = 0;

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
        rightFlickerPos = flicker1down;
        backFlickerPos = flicker2down;
        leftFlickerPos = flicker3down;
        targetTps = 0;
        hoodPos = closeHood;
        autoTurret = false;
        canShoot = false;
        pinpoint.resetPosAndIMU();
    }
}
