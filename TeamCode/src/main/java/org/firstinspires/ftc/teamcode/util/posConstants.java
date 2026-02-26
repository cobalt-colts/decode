package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

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
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import dev.nextftc.ftc.ActiveOpMode;

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
    public static Servo hood, flicker3, flicker2, flicker1, light3, light2, light1;
    public static AnalogInput rightAnalog, backAnalog, leftAnalog;
    public static DigitalChannel magnet;
    public static Limelight3A limelight;
    public static GoBildaPinpointDriver pinpoint;
    public static PIDFController controller;
    public static PredominantColorProcessor sensor1 = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.RED, PredominantColorProcessor.Swatch.BLUE, PredominantColorProcessor.Swatch.YELLOW, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
            .build();
    public static PredominantColorProcessor sensor2 = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, .1, 0.9, -0.3))
            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.RED, PredominantColorProcessor.Swatch.BLUE, PredominantColorProcessor.Swatch.YELLOW, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
            .build();
    public static PredominantColorProcessor sensor3 = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.4, 0.45, -0.15, 0.2))
            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.RED, PredominantColorProcessor.Swatch.BLUE, PredominantColorProcessor.Swatch.YELLOW, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
            .build();
    public static VisionPortal portal = new VisionPortal.Builder().addProcessor(sensor1).addProcessor(sensor2).addProcessor(sensor3)
            .setCameraResolution(new Size(640, 480))
            .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "internalcam"))
            .build();
    public static PredominantColorProcessor.Result result1, result2, result3;
//    public static GoBildaPrismDriver prism;

    // INTAKE
    public static double intakeIn = 1;
    public static double intakeOut = -1; //-.5

    // INDEX

    // flicker 1
    public static final double flicker1down = .24; //.22
    public static final double flicker1up = .6;
    public static double flicker1DownThreshold = 0.92;
    public static double flicker1UpThreshold = 1.8;

    //flicker 2
    public static double flicker2down = 0.77; // 0.375
    public static final double flicker2up = 0.38;
    public static double flicker2DownThreshold = 2.15;
    public static double flicker2UpThreshold = 1.5;

    // flicker 3
    public static final double flicker3down = 0.68;
    public static final double flicker3up = .31;
    public static double flicker3DownThreshold = 1.8; //2.1
    public static double flicker3UpThreshold = 1.4; // 1.19

    public static double red = 0.277;
    public static double yellow = 0.388;
    public static double green = 0.5;
    public static double blue = 0.64;
    public static double purple = 0.722;

    // SHOOTER
    public static final double closeSpeed = 746.67;
    public static final double closeHood = 0.25; // 0.28
    public static final double farSpeed = 2000;
    public static final double farHood = 0.19; // 0.19
    public static double hoodAdjust = 0.03;
    public static double hoodAdjustClose = 0.02;
    public static double hoodAdjustFar = 0.04;
    public static final double flywheelThreshold = 0.1;
    public static final double turretThreshold = 0.1;
    public static final double turretManual = 0.1; // 0.35
    public static final int turretMax = 300;
    public static final int turretMin = -100;
    public static final int limelightSlow = 250;
    public static final int limelightFast = 150; // 100
    public static double farAngleOffset = 5; // 5
    public static double tolerance = 1;
    public static double ticksPerDegree = .75;
    public static double yawConstant = -1;
    public static double turretP = 100;

    // auto turret pos constants

    public static int redGoalInit = 90; // Was 68
    public static int redFarInit = 25;
    public static int redFarPickup = -88; //-90
    public static int redGoalPickup = -65; //-65
    public static int redGoalPark = -39;

    public static int blueGoalInit = -redGoalInit; // Was 68
    public static int blueFarInit = -redFarInit;
    public static int blueFarPickup = -redFarPickup; //-90
    public static int blueGoalPickup = -redGoalPickup; //-65
    public static int blueGoalPark = -redGoalPark;


}
