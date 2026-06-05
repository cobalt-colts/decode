package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
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

@Config
//@Configurable
@SuppressWarnings("CannotResolve")
public class posConstants {;
    public static DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intake;
    public static DcMotorEx turret, thrower1, thrower2;
    public static Servo hood, flicker3, flicker2, flicker1, light1, light2, light3;
    public static AnalogInput rightAnalog, backAnalog, leftAnalog;
    public static DigitalChannel magnet;

    // In posConstants, change the sensor declarations to just:
    public static PredominantColorProcessor sensor1;
    public static PredominantColorProcessor sensor2;
    public static PredominantColorProcessor sensor3;

    // Add this static method:
    public static void rebuildSensors() {
        sensor1 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, .1, 0.9, -0.3))
                .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
                .build();
        sensor2 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
                .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
                .build();
        sensor3 = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, -0.2, 0.2))
                .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
                .build();
    }
//    public static PredominantColorProcessor sensor2 = new PredominantColorProcessor.Builder()
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
//            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
//            .build();
//    public static PredominantColorProcessor sensor1= new PredominantColorProcessor.Builder()
//            .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, .1, 0.9, -0.3))
//            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
//            .build();
//    public static PredominantColorProcessor sensor3 = new PredominantColorProcessor.Builder()
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, -0.2, 0.2))
//            .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.WHITE)
//            .build();
    public static VisionPortal portal;
    public static Limelight3A limelight;
    public static GoBildaPinpointDriver pinpoint;
    public static PIDFController controller;
//    public static GoBildaPrismDriver prism;

    // INTAKE
    public static double intakeIn = 1;
    public static double intakeOut = -1; //-.5

    // INDEX

    // flicker 1
    public static final double flicker1down = .25; //.24     .22
    public static final double flicker1up = .8; //.6
    public static double flicker1DownThreshold = 0.92;
    public static double flicker1UpThreshold = 1.8;

    //flicker 2
    public static double flicker2down = 0.77; // 0.375
    public static final double flicker2up = 0.38;
    public static final double flicker2transfer = 0.72;
    public static double flicker2DownThreshold = 2.15;
    public static double flicker2UpThreshold = 1.8;

    // flicker 3
    public static final double flicker3down = 0.68;
    public static final double flicker3up = .31;
    public static double flicker3DownThreshold = 1.8; //2.1
    public static double flicker3UpThreshold = 1.1; // 1.19
    public static int liftUpTime = 60;
    public static int liftDownTime = 45;
    public static final double greenThreshold = 450; //500
    public static final double blueThreshold = 500; //550

    // SHOOTER
    public static final double closeSpeed = 746.67;
    public static final double closeHood = 0.25; // 0.28
    public static final double farSpeed = 2000;
    public static final double farHood = 0.03; // 0.08
    public static double hoodAdjust = 0.03;
    public static double hoodAdjustClose = 0.02;
    public static double hoodAdjustFar = 0.04;
    public static final double flywheelThreshold = 0.1;
    public static final double turretThreshold = 0.1;
    public static final double turretManual = 0.1; // 0.35
    public static final int turretMax = 300;
    public static final int turretMin = -170;
    public static final double turretFarAdjust = 7.5; //5
    public static final int limelightSlow = 250;
    public static final int limelightFast = 150; // 100
    public static double tolerance = 0.4;
    public static double ticksPerDegree = .75;
    public static double turretP = 50;
    public static double turretRunToPositionPower = 0.75;

    // Teleop Limelight tx aiming
    public static int redTeleopPipeline = 2;
    public static int blueTeleopPipeline = 3;
    public static double limelightTxDirection = 1.0;
    public static double turretAimDirection = 1.0;
    public static double turretAimOffsetTicks = 0.0;
    public static double turretAimCameraOffsetDeg = 0.0;
    public static double limelightAimMaxStalenessMs = 160;
    public static double limelightAimLostHoldMs = 500;
    public static double limelightAimTxDeadbandDeg = 0.35;
    public static double limelightAimMaxTxDeg = 25.0;
    public static double limelightAimFilterAlpha = 0.25;
    public static double limelightAimMaxStepTicks = 10.0;
    public static double limelightAimMaxCorrectionTicks = 70.0;
    public static boolean useLimelightTxTurretAim = true;
    public static double redTeleopForwardHeadingDeg = 90.0;
    public static double blueTeleopForwardHeadingDeg = -90.0;

    // auto turret pos constants

    public static int redGoalInit = 90; // Was 68
    public static int redFarObelisk = 0;
    public static int redFarInit = 28; //25    32
    public static int redFarPickup = -90; //-92     -88   -96
    public static int redGoalPickup = -65; //-65
    public static int redGoalPark = -35; //-39

    public static int blueFarInit = -redFarInit;
    public static int blueFarPickup = -redFarPickup;
    public static int blueGoalPickup = -redGoalPickup;
    public static int blueGoalInit = -redGoalInit;

    public static int blueGoalPark = -redGoalPark;


}
