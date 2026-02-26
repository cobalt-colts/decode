package org.firstinspires.ftc.teamcode.util;

//import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.indexPower;

import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import  static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.*;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.CORNERM3RedFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

        import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import kotlin.ParameterName;

@Config
public class subsystems {
    public static boolean start = false;
    public static char motifOrder[] = {'b', 'b', 'b'};
    public static ArrayList<Character> indexOrder = new ArrayList<>(3);
    public static int count = 0;
    public static int matchingSpot = -1;
    public static boolean hasAnyBalls = false;

    public static enum motifs {
        PPG,
        PGP,
        GPP,
        NONE
    }

    public static motifs motif = motifs.NONE;


    public static String flickOrder = "";
    public static ServoEx[] flickers = new ServoEx[3];

    public static class ColorSensing implements Subsystem {

        public static final ColorSensing INSTANCE = new ColorSensing();

        private ColorSensing() {
        }

        public PredominantColorProcessor sensor1;
        public PredominantColorProcessor sensor2;
        public PredominantColorProcessor sensor3;
        public VisionPortal portal;

        public static PredominantColorProcessor.Result result1;
        public static PredominantColorProcessor.Result result2;
        public static PredominantColorProcessor.Result result3;

        public static boolean[] isoccupied = new boolean[3];

        private long lastToggleTime = 0;
        private boolean lightsOn = false;
        private static final long BLINK_INTERVAL_MS = 500;

        // Add this:
//        public static boolean hasAnyBalls() { return (isoccupied[0] || isoccupied[1] || isoccupied[2]); }

        private static boolean isball(PredominantColorProcessor.Result result) {
            if (result == null) return false;
            PredominantColorProcessor.Swatch resultswatch = result.closestSwatch;
            return resultswatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN
                    || resultswatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE;
        }

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            result1 = sensor1.getAnalysis();
            result2 = sensor2.getAnalysis();
            result3 = sensor3.getAnalysis();

            isoccupied[0] = result1 != null && isball(result1);
            isoccupied[1] = result2 != null && isball(result2);
            isoccupied[2] = result3 != null && isball(result3);

            hasAnyBalls = (isoccupied[0] || isoccupied[1] || isoccupied[2]);

            // Set the RGB lights based on what colors are seen:
            // No Artifact = red = 0.277
            // Artifact = purple=0.7 or green=0.5, specific to the position represented
            // Shooter up to speed Thrower.INSTANCE.atvelocity and aimed at goal Turret.INSTANCE.atposition = blinking. Set the light to 0.0 in the shooter's periodic() and hope there's enough lag to "blink"
            long now = System.nanoTime() / 1_000_000;
//            if (now - lastToggleTime >= BLINK_INTERVAL_MS) { // build error, miles, 2/25/26, 7:18:32 PM
//                lightOn = !lightOn;
//                lastToggleTime = now;
//
//                if (lightOn) {
//                        // Turn on the 3 lights.
//                } else {
//                        if (Thrower.INSTANCE.atvelocity && Turret.INSTANCE.atposition) {
//                                // We are at speed, so we want to blink. Turn off the lights (set all 3 to 0)
//                        }
//                }
//                led.setState(lightOn);  // whatever your LED control is
//            }
        }

        @Override
        public void initialize()  {
            Subsystem.super.initialize();

            sensor1 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.RED,
                            PredominantColorProcessor.Swatch.BLUE,
                            PredominantColorProcessor.Swatch.YELLOW,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();

            sensor2= new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, .1, 0.9, -0.3))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.RED,
                            PredominantColorProcessor.Swatch.BLUE,
                            PredominantColorProcessor.Swatch.YELLOW,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();
            sensor3 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.4, 0.45, -0.15, 0.2))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.RED,
                            PredominantColorProcessor.Swatch.BLUE,
                            PredominantColorProcessor.Swatch.YELLOW,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();
            portal = new VisionPortal.Builder()
                    .addProcessor(sensor1)
                    .addProcessor(sensor2)
                    .addProcessor(sensor3)
                    .setCameraResolution(new Size(640, 480))
                    .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "internalcam"))
                    .build();
        }

    }
    public static class Camera implements Subsystem {
        public static final Camera INSTANCE = new Camera();

        private Camera() {
        }


        motifs getMotif(List<Integer> tags) {
            if (tags.contains(21) && tags.contains(22)) {
                motifOrder[0] = 'g';
                motifOrder[1] = 'p';
                motifOrder[2] = 'p';
                return motifs.GPP;
            } else if (tags.contains(22) && tags.contains(23)) {
                motifOrder[0] = 'p';
                motifOrder[1] = 'g';
                motifOrder[2] = 'p';
                return motifs.PGP;
            } else if (tags.contains(23) && tags.contains(21)) {
                motifOrder[0] = 'p';
                motifOrder[1] = 'p';
                motifOrder[2] = 'g';
                return motifs.PPG;
            } else return motifs.GPP;
        }

        Limelight3A limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        public LambdaCommand setmotif = new LambdaCommand()
                .setStart(() -> {
                    limelight.setPollRateHz(100);
                    limelight.pipelineSwitch(1);
                    limelight.start();
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        List<Integer> tags = new ArrayList<Integer>();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId(); // The ID number of the fiducial
                            tags.add(id);
                            motif = getMotif(tags);
                            ActiveOpMode.telemetry().addData("motif", motif);
                        }
                        motif = getMotif(tags);
                    }
                })
                .setInterruptible(false)
                .requires(this)
                .setIsDone(() -> motif != motifs.NONE);
    }

    public static class Index implements Subsystem {
        public static final Index INSTANCE = new Index();

        private Index() {
        }

        // Actuators
        private final ServoEx flicker1 = new ServoEx("flicker1");
        private final ServoEx flicker2 = new ServoEx("flicker2");
        private final ServoEx flicker3 = new ServoEx("flicker3");


        public Command launch1 = new SequentialGroup(
                new SetPosition(flicker1, flicker1up),
                new Delay(0.2),
                new SetPosition(flicker1, flicker1down)
        );
        public Command launch2 = new SequentialGroup(
                new SetPosition(flicker2, flicker2up),
                new Delay(0.2),
                new SetPosition(flicker2, flicker2down)
        );
        public Command launch3 = new SequentialGroup(
                new SetPosition(flicker3, flicker3up),
                new Delay(0.2),
                new SetPosition(flicker3, flicker3down)
        );

        public final Command alldown = new ParallelGroup(
                new SetPosition(flicker1, flicker1down),
                new SetPosition(flicker2, flicker2down),
                new SetPosition(flicker3, flicker3down)
        );

        public Command closeunsortedlaunch = new SequentialGroup(
                launch2,
                new InstantCommand(() -> {
                    Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);
                }),
                new Delay(0.25),
                launch1,
                new Delay(0.25),
                launch3,
                new InstantCommand(() -> {
                    Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);
                }),
                new Delay(0.25)

        );
        // Merge these two launches to have one with ifelse
//        public Command secondcloseunsortedlaunch = new SequentialGroup(
//                new IfElseCommand(
//                        () -> ColorSensing.hasAnyBalls(),
//                        new SequentialGroup(
//                                launch2,
//                                new InstantCommand(() -> {
//                                    Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);
//                                }),
//                                new Delay(0.25),
//                                launch1,
//                                new Delay(0.25),
//                                launch3,
//                                new InstantCommand(() -> {
//                                    Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);
//                                }),
//                                new Delay(0.25)
//                        )
//                )
//        );

        private int index = 0;
        public LambdaCommand flickerOrder = new LambdaCommand()
                .setStart(() -> {
                    switch (motif) {
                        case GPP:
                            if (indexOrder.contains('g')) {
                                if (indexOrder.indexOf('g') == indexOrder.lastIndexOf('g')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('g'));
                                    flickOrder += indexOrder.indexOf('p');
                                    flickOrder += indexOrder.lastIndexOf('p');
                                } else if (indexOrder.contains('p')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('g'));
                                    flickOrder += indexOrder.lastIndexOf('g');
                                    flickOrder += indexOrder.indexOf('p');
                                } else {
                                    flickOrder = "123";
                                }
                            } else {
                                flickOrder = "123";
                            }
                            break;

                        case PGP:
                            if (indexOrder.contains('g')) {
                                if (indexOrder.indexOf('g') == indexOrder.lastIndexOf('g')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('p'));
                                    flickOrder += indexOrder.indexOf('g');
                                    flickOrder += indexOrder.lastIndexOf('p');
                                } else if (indexOrder.contains('p')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('g'));
                                    flickOrder += indexOrder.lastIndexOf('g');
                                    flickOrder += indexOrder.indexOf('p');
                                } else {
                                    flickOrder = "123";
                                }
                            } else {
                                flickOrder = "123";
                            }
                            break;

                        case PPG:
                            if (indexOrder.contains('g')) {
                                if (indexOrder.indexOf('g') == indexOrder.lastIndexOf('g')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('p'));
                                    flickOrder += indexOrder.lastIndexOf('p');
                                    flickOrder += indexOrder.indexOf('g');
                                } else if (indexOrder.contains('p')) {
                                    flickOrder = Integer.toString(indexOrder.indexOf('p'));
                                    flickOrder += indexOrder.lastIndexOf('g');
                                    flickOrder += indexOrder.indexOf('g');
                                } else {
                                    flickOrder = "123";
                                }
                            } else {
                                flickOrder = "123";
                            }
                            break;

                        case NONE:
                            flickOrder = "123";
                            break;
                    }

                })
                .setIsDone(() -> true);
        ;

        public LambdaCommand closesortedLaunch = new LambdaCommand()
                .setStart(() -> {
                    switch (flickOrder) {
                        case "123":
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;

                        case "132":
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;

                        case "213":
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;

                        case "231":
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;

                        case "312":
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;

                        case "321":
                            new InstantCommand(launch3);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);});
                            new Delay(0.25);
                            new InstantCommand(launch2);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.35);});
                            new Delay(0.25);
                            new InstantCommand(launch1);
                            new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);});
                            new Delay(0.25);
                            break;
                    }

                })
                .requires(this, Camera.INSTANCE, launch3, launch2, launch1)
                .setIsDone(() -> true);
        ;

        public Command farunsortedlaunch = new SequentialGroup(
                launch2,
                new Delay(1),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                launch1,
                new Delay(1),
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                launch3

        );

    }
    public static class Intake implements Subsystem {
        public static final Intake INSTANCE = new Intake();

        private Intake() {
        }

        private DcMotorEx intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
        double intakePower = -1;
        public static boolean negative = false;

        public Command outtake = new InstantCommand(() -> {
            intakeMotor.setPower(0.5);
        });
        public Command intake = new InstantCommand(() -> {
            intakeMotor.setPower(-1);
        });

        @Override
        public void periodic() {
            Subsystem.super.periodic();
//            if (start) {
//                intakePower = (negative ? -1 : 0.5);
//                intakeMotor.setPower(intakePower);
//            }
        }
    }

    public static class Thrower implements Subsystem {
        public static final Thrower INSTANCE = new Thrower();

        public static double targetvelocity = 1370; // Starting value, good for near auto

        private double velocity = 0;    // What is this variable for?

        private Thrower() {
        }

        public boolean atvelocity = false;
        public boolean isshooteron = false;

        DcMotorEx thrower1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower2");
        Servo hood = ActiveOpMode.hardwareMap().get(Servo.class, "hood");

        public static PIDFController controller;
        public static Limelight3A limelight;

        public static double hoodpos = .3;  // Initial value, somewhat reasonable for near auto

        public Command shooteron = new InstantCommand(() -> {
            isshooteron = true;
        });
        public Command shooteroff = new InstantCommand(() -> {
            isshooteron = false;
        });

        public void initialize() {
            Subsystem.super.initialize();
            isshooteron = false;
            controller = new PIDFController(kP, kI, kD, kF);

            limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
            limelight.start();

            thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            thrower1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            thrower1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            thrower2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            hood.setPosition(0.5);

            //thrower1.setVelocity(0);
            //thrower2.setPower(thrower1.getPower());
        }

        public void periodic() {
            Subsystem.super.periodic();
            if (start) {

                isshooteron = true;

                // These two override everything done externally....
//                  targetvelocity = 1350; // This seems to be the only one used. //1500
//                hoodpos = .25;

//                targetvelocity = ll.fetchFlywheelSpeed(limelight);
//                hoodpos = ll.fetchHoodPos(limelight);

                if (!Double.isNaN(ll.fetchFlywheelSpeed(limelight))) { targetvelocity = ll.fetchFlywheelSpeed(limelight); }
                if (!Double.isNaN(ll.fetchHoodPos(limelight))) { hoodpos = ll.fetchHoodPos(limelight); }

                atvelocity = (targetvelocity) - (thrower1.getVelocity() / 28) * 60 <= 10;

                double currentVelocity = (thrower1.getVelocity() / 28) * 60;
                double pid = controller.calculate(currentVelocity, targetvelocity);

                if (targetvelocity < 100) {
                    thrower1.setMotorDisable();
                    thrower2.setMotorDisable();
                } else {
                    if (isshooteron) {
                        thrower1.setPower(pid);
                        thrower2.setPower(pid);
                    }
                }

                hood.setPosition(hoodpos);

//                ActiveOpMode.telemetry().addData("thrower1vel - velocity: ", Math.abs(Math.abs(thrower1.getVelocity()) - Math.abs(velocity)));
                ActiveOpMode.telemetry().addData("atvelocity: ", atvelocity);
                ActiveOpMode.telemetry().addData("targetvelocity ", targetvelocity);
                ActiveOpMode.telemetry().addData("velocity/28*60 ", (thrower1.getVelocity() / 28) * 60);

                // Do not remove this one unless you check with Crocker:
                if (thrower1.getVelocity() == 0 || thrower2.getVelocity() == 0) {
                    ActiveOpMode.telemetry().addLine("ENCODER CABLE UNPLUGGED???");
                }
// rely on later telemetry to do the update() so it's on one screen
//                ActiveOpMode.telemetry().update();

                TelemetryManager.TelemetryWrapper panelstel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

                panelstel.addData("targetvelocity ", targetvelocity);
                panelstel.addData("thrower1/28*60", (thrower1.getVelocity() / 28) * 60);
                panelstel.addData("hoodpos", hoodpos);
                panelstel.addData("atvelocity", atvelocity);
                panelstel.addData("error", targetvelocity - (thrower1.getVelocity() / 28) * 60);
                panelstel.update();


            } else {
                Index.INSTANCE.alldown.schedule();
                thrower1.setPower(0);
                thrower2.setPower(0);
                velocity = 1300; // What is this variable for?
//                ActiveOpMode.telemetry().addLine("NOT RUNNING");
//                ActiveOpMode.telemetry().addData("target ", targetTps);
//                ActiveOpMode.telemetry().addData("velocity", thrower1.getVelocity());
//                ActiveOpMode.telemetry().update();

            }
        }
    }

    public static class Turret implements Subsystem {


        public static final Turret INSTANCE = new Turret();
        public static int turretTargetPos = 0;

        private Turret() {
        }

        public boolean atposition = false;

        public static DcMotorEx turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        public static int initPos = redFarInit;

        public Command redgoalinit = new InstantCommand(() -> {
            turretTargetPos = redGoalInit;
        });
        public Command redfarinit = new InstantCommand(() -> {
            turretTargetPos = redFarInit;
        });
        public Command redfar = new InstantCommand(() -> {
            turretTargetPos = redFarPickup;
        });
        public Command redgoal = new InstantCommand(() -> {
            turretTargetPos = redGoalPickup;
        });

        public Command redpark = new InstantCommand(() -> {
            turretTargetPos = redGoalPark;
        });

        public Command blueinit = new InstantCommand(() -> {
            turretTargetPos = blueFarInit;
        });
        public Command bluefar = new InstantCommand(() -> {
            turretTargetPos = blueFarPickup;
        });
        public Command bluegoal = new InstantCommand(() -> {
            turretTargetPos = blueGoalPickup;
        });

        public Command home = new InstantCommand(() -> {
            turretTargetPos = 0;
        });


//        public Command redinit = new LambdaCommand()
//                .setStart(() -> {turretTargetPos = redFarInit;
//                })
//                .setIsDone(() -> Math.abs(Math.abs(turret.getCurrentPosition()) - Math.abs(turretTargetPos)) <= 2);


        public void initialize() {
            Subsystem.super.initialize();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setTargetPosition(initPos);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPositionPIDFCoefficients(100);
//            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turretTargetPos = initPos;
//            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        public void periodic() {
            Subsystem.super.periodic();
            turret.setTargetPosition(turretTargetPos);
            turret.setPower(1);
            atposition = (Math.abs(turret.getCurrentPosition() - turretTargetPos) <= 2);

            ActiveOpMode.telemetry().addData("turretTargetPos", turretTargetPos);
            ActiveOpMode.telemetry().addData("turretCurrentPos", turret.getCurrentPosition());
            ActiveOpMode.telemetry().addData("turretError", turretTargetPos - turret.getCurrentPosition());
            ActiveOpMode.telemetry().addData("turretPower", turret.getPower());
            ActiveOpMode.telemetry().addData("turretAtPosition", atposition);
            ActiveOpMode.telemetry().addData("hoodpos", Thrower.hoodpos);
            ActiveOpMode.telemetry().addData("motif", motif);
            ActiveOpMode.telemetry().addLine("--- COLOR SENSING ---");
            ActiveOpMode.telemetry().addData("isOccupied[0]", ColorSensing.isoccupied[0]);
            ActiveOpMode.telemetry().addData("isOccupied[1]", ColorSensing.isoccupied[1]);
            ActiveOpMode.telemetry().addData("isOccupied[2]", ColorSensing.isoccupied[2]);
            ActiveOpMode.telemetry().addData("swatch[0]", ColorSensing.result1 != null ? ColorSensing.result1.closestSwatch : "null");
            ActiveOpMode.telemetry().addData("swatch[1]", ColorSensing.result2 != null ? ColorSensing.result2.closestSwatch : "null");
            ActiveOpMode.telemetry().addData("swatch[2]", ColorSensing.result3 != null ? ColorSensing.result3.closestSwatch : "null");
            ActiveOpMode.telemetry().update();
        }

    }
}

