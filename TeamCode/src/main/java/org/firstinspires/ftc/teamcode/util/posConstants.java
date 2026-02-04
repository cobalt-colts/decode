package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
//
//import org.firstinspires.ftc.teamcode.Prism.Color;
//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@Config
@Configurable
@SuppressWarnings("CannotResolve")
public class posConstants {;
    public static DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intake;
    public static DcMotorEx turret, thrower1, thrower2;
    public static Servo hood, leftFlicker, backFlicker, rightFlicker;
    public static ColorSensor right1, right2, back1, back2, left1, left2;
    public static AnalogInput rightAnalog, backAnalog, leftAnalog;
    public static DigitalChannel turretMag;
    public static Limelight3A limelight;
    public static GoBildaPinpointDriver pinpoint;
//    public static GoBildaPrismDriver prism;

    // INTAKE
    public static double intakeIn = 1;
    public static double intakeOut = -1; //-.5

    // INDEX
    public static final double rightFlickerDown = 0.65;
    public static final double rightFlickerUp = 0.39;
    public static double rightFlickerDownThreshold = 1.68;
    public static double rightFlickerUpThreshold = 2.0;
    public static double backFlickerDown = 0.35; // 0.375
    public static final double backFlickerUp = 0.65;
    public static double backFlickerDownThreshold = 1.68;
    public static double backFlickerUpThreshold = 2.0;
    public static final double leftFlickerDown = 0.375;
    public static final double leftFlickerUp = 0.65;
    public static double leftFlickerDownThreshold = .5;
    public static double leftFlickerUpThreshold = 1.0;
    public static int liftUpTime = 60;
    public static int liftDownTime = 45;
    public static final double greenThreshold = 450; //500
    public static final double blueThreshold = 500; //550

    // SHOOTER
    public static final double closeSpeed = 746.67;
    public static final double closeHood = 0.25; // 0.28
    public static final double farSpeed = 2000;
    public static final double farHood = 0.19; // 0.21
    public static double hoodAdjust = 0.03;
    public static double hoodAdjustClose = 0.02;
    public static double hoodAdjustFar = 0.04;
    public static final double flywheelThreshold = 0.1;
    public static final double turretThreshold = 0.1;
    public static final double turretManual = 0.1; // 0.35
    public static final int turretMax = 116;
    public static final int turretMin = -156;
    public static final int limelightSlow = 250;
    public static final int limelightFast = 150; // 100
    public static double farAngleOffset = 5; // 5
    public static double tolerance = 1;
    public static double ticksPerDegree = .75;
    public static double turretP = 100;

    // auto turret pos constants

    public static int redFarObelisk = 0;
    public static int redFarInit = 25;
    public static int redFarPickup = -88; //-90
    public static int redGoalPickup = 10;

    public static int blueFarInit = -redFarInit;
    public static int blueFarPickup = -redFarPickup;
    public static int blueGoalPickup = -redGoalPickup;


}
