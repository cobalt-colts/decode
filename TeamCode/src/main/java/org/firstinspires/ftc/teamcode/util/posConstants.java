package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class posConstants {
    public static DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intake;
    public static DcMotorEx turret, thrower1, thrower2;
    public static Servo hood, lift1, lift2, lift3;
    public static ColorSensor color11, color12, color21, color22, color31, color32;
    public static AnalogInput lift1Analog, lift2Analog, lift3Analog;
    public static DigitalChannel turretMag;
    public static Limelight3A limelight;
    public static IMU imu;

    // INTAKE
    public static double intakeIn = 1;
    public static double intakeOut = -.5;

    // INDEX
    public static double lift1Down = 0.5;
    public static double lift1Up = 0.5;
    public static double lift1UpThreshold = 0.5;
    public static double lift1DownThreshold = 0.5;
    public static double lift2Down = 0.5;
    public static double lift2Up = 0.5;
    public static double lift2UpThreshold = 0.5;
    public static double lift2DownThreshold = 0.5;
    public static double lift3Down = 0.5;
    public static double lift3Up = 0.5;
    public static double lift3UpThreshold = 0.5;
    public static double lift3DownThreshold = 0.5;
    public static int liftUpTime = 60;
    public static int liftDownTime = 45;
    public static double greenThreshold = 50;
    public static double blueThreshold = 50;

    // SHOOTER
    public static double closeSpeed = 746.67;
    public static double closeHood = 0.3; // 0.25   0.35
    public static double farSpeed = 1073.33;
    public static double farHood = 0.21; // 0.1
    public static double flywheelThreshold = 0.1;
    public static double turretThreshold = 0.1;
    public static double turretManual = 0.35; // 0.25
    public static int turretMax = 200;
    public static int turretMin = -80;
    public static int limelightSlow = 250;
    public static int limelightFast = 100;
}
