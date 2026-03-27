package org.firstinspires.ftc.teamcode.util;

//import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.indexPower;

import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import  static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.*;
import static org.firstinspires.ftc.teamcode.util.positions.*;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

        import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.conditionals.SwitchCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
public class subsystems {
    private static final Logger log = LoggerFactory.getLogger(subsystems.class);
    public static boolean start = false;
    public static boolean teleop = false;
    public static char motifOrder[] = {'b', 'b', 'b'};
    public static ArrayList<Character> indexOrder = new ArrayList<>(3);
    public static int count = 0;
    public static int matchingSpot = -1;
    public static boolean hasAnyBalls = false;
    public static boolean far = false;

    public static enum motifs {
        PPG,
        PGP,
        GPP,
        NONE
    }

    public static motifs motif = motifs.NONE;


    public static String flickOrder = "321";
    public static ServoEx[] flickers = new ServoEx[3];

    public static boolean[] isoccupied = new boolean[3];


    public static class ColorSensing implements Subsystem {

        public static final ColorSensing INSTANCE = new ColorSensing();

        private Servo light1;
        private Servo light2;
        private Servo light3;

        private ColorSensing() {
        }

        public PredominantColorProcessor sensor1;
        public PredominantColorProcessor sensor2;
        public PredominantColorProcessor sensor3;
        public VisionPortal portal;

        public static PredominantColorProcessor.Result result1;
        public static PredominantColorProcessor.Result result2;
        public static PredominantColorProcessor.Result result3;

        private long lastToggleTime = 0;
        private boolean lightsOn = false;
        private static final long BLINK_INTERVAL_MS = 250;

//        public static boolean hasAnyBalls() { return (isoccupied[0] || isoccupied[1] || isoccupied[2]); }


        private static final double ALPHA = 0.15; // tune: lower = smoother but slower to respond
        private float[] satAvg;
        private boolean[] initialized;

        private void updateOccupied(int i, PredominantColorProcessor.Result result, long satCutoff) {
            if (result == null) return;

            float sat = result.HSV[1];

            // Initialize on first reading
            if (!initialized[i]) {
                satAvg[i] = sat;
                initialized[i] = true;
            } else {
                satAvg[i] = (float)(ALPHA * sat + (1 - ALPHA) * satAvg[i]);
            }

            // Use a hysteresis band to prevent flickering at the threshold
            if (isoccupied[i]) {
                // Only mark empty if average drops well below cutoff
                if (satAvg[i] < satCutoff - 15) isoccupied[i] = false;
            } else {
                // Only mark occupied if average rises well above cutoff
                if (satAvg[i] > satCutoff + 15) isoccupied[i] = true;
            }

            // Expose for telemetry tuning
            ActiveOpMode.telemetry().addData("satAvg[" + i + "]", satAvg[i]);
        }

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            result1 = sensor1.getAnalysis();
            result2 = sensor2.getAnalysis();
            result3 = sensor3.getAnalysis();

            updateOccupied(0, result1, GREY_SATURATION1);
            updateOccupied(1, result2, GREY_SATURATION2);
            updateOccupied(2, result3, GREY_SATURATION3);
//            ActiveOpMode.telemetry().addData("sat1", result1.HSV[1]);
//            ActiveOpMode.telemetry().addData("sat2", result2.HSV[1]);
//            ActiveOpMode.telemetry().addData("sat3", result3.HSV[1]);

//            ActiveOpMode.telemetry().addData("hue0", result1.HSV[0]);
//            ActiveOpMode.telemetry().addData("hue1", result2.HSV[0]);
//            ActiveOpMode.telemetry().addData("hue2", result3.HSV[0]);

            // Update our index of what color is in what slot so we can sort later:
            indexOrder.clear();
            PredominantColorProcessor.Result[] results = {result1, result2, result3};
            long[] satCutoffs = {GREY_SATURATION1, GREY_SATURATION2, GREY_SATURATION3};

            for (int i = 0; i < 3; i++) {
                PredominantColorProcessor.Result result = results[i];
                if (isoccupied[i]) {
                    // Use isball with hue check to confirm color, same logic as occupation detection
                    if (result != null && result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN
                            && isball(result, satCutoffs[i])) {
                        indexOrder.add('g');
                    } else {
                        indexOrder.add('p');
                    }
                } else {
                    indexOrder.add('b');
                }
            }

            hasAnyBalls = (isoccupied[0] || isoccupied[1] || isoccupied[2]);

//            if (!hasAnyBalls) new InstantCommand(Intake);
            if (start) {
                if (isoccupied[0] && isoccupied[1] && isoccupied[2]) {
                    Intake.INSTANCE.intakeMotor.setPower(0.5);
                } else {
                    Intake.INSTANCE.intakeMotor.setPower(-1);
                }
            }

            // Set the RGB lights based on what colors are seen:
            // Artifact = purple=0.7 or green=0.5, specific to the position represented
            // Shooter up to speed Thrower.INSTANCE.atvelocity and aimed at goal Turret.INSTANCE.atposition = blinking. Set the light to 0.0 in the shooter's periodic() and hope there's enough lag to "blink"
            long now = System.nanoTime() / 1000000;
            if (now - lastToggleTime >= BLINK_INTERVAL_MS) {
                lightsOn = !lightsOn;
                lastToggleTime = now;

                if (lightsOn) {
                    // Turn on the 3 lights.
                    light1.setPosition(colorToRGBServo(ColorSensing.result1, GREY_SATURATION1));
                    light2.setPosition(colorToRGBServo(ColorSensing.result2, GREY_SATURATION2));
                    light3.setPosition(colorToRGBServo(ColorSensing.result3, GREY_SATURATION3));
                } else {
//                    if (Thrower.INSTANCE.atvelocity && Turret.INSTANCE.atposition) {
//                        light1.setPosition(.1);
//                        light2.setPosition(.1);
//                        light3.setPosition(.1);
//                    }
                }
            }
        }

        @Override
        public void initialize()  {
            Subsystem.super.initialize();

            initialized = new boolean[3];
            satAvg = new float[3];

            light1 = ActiveOpMode.hardwareMap().get(Servo.class, "light1");
            light2 = ActiveOpMode.hardwareMap().get(Servo.class, "light2");
            light3 = ActiveOpMode.hardwareMap().get(Servo.class, "light3");

            sensor2 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();

            sensor1= new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(0.4, .3, 0.9, -0.3))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();
            sensor3 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.4, 0.6, -0.2, 0.3))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
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


//        motifs getMotif(List<Integer> tags) {
//            if (tags.contains(21) && tags.contains(22)) {
//                motifOrder[0] = 'g';
//                motifOrder[1] = 'p';
//                motifOrder[2] = 'p';
//                return motifs.GPP;
//            } else if (tags.contains(22) && tags.contains(23)) {
//                motifOrder[0] = 'p';
//                motifOrder[1] = 'g';
//                motifOrder[2] = 'p';
//                return motifs.PGP;
//            } else if (tags.contains(23) && tags.contains(21)) {
//                motifOrder[0] = 'p';
//                motifOrder[1] = 'p';
//                motifOrder[2] = 'g';
//                return motifs.PPG;
//            } else return motifs.GPP;
//        }

        motifs getMotif(List<LLResultTypes.FiducialResult> fiducials) {
            if (fiducials.size() < 2) return motifs.NONE;

            // Sort by tx: most negative = leftmost
            fiducials.sort((a, b) -> Double.compare(a.getTargetXDegrees(), b.getTargetXDegrees()));

            int leftTag  = fiducials.get(0).getFiducialId();
            int rightTag = fiducials.get(1).getFiducialId();

            if (redAlliance) {
                if (leftTag == 23 && rightTag == 22) return motifs.PPG;
                if (leftTag == 22 && rightTag == 21) return motifs.PGP;
                if (leftTag == 21 && rightTag == 23) return motifs.GPP;
            } else {
                if (leftTag == 21 && rightTag == 23) return motifs.PPG;
                if (leftTag == 22 && rightTag == 21) return motifs.GPP;
                if (leftTag == 23 && rightTag == 22) return motifs.PGP;
            }

            return motifs.NONE;
        }

        Limelight3A limelight; // = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        @Override
        public void initialize() {
            Subsystem.super.initialize();

            limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");;
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(1);
            limelight.start(); // add this

        }

//        public LambdaCommand setmotif = new LambdaCommand()
//                .setStart(() -> {
//                    limelight.setPollRateHz(100);
//                    limelight.pipelineSwitch(1);
//                    limelight.start();
//                    LLResult result = limelight.getLatestResult();
//                    if (result != null && result.isValid()) {
//                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//                        List<Integer> tags = new ArrayList<Integer>();
//                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//                            tags.add(id);
//                            motif = getMotif(tags);
//                            ActiveOpMode.telemetry().addData("motif", motif);
//                        }
//                        motif = getMotif(tags);
//                    }
//                    if (motif == motifs.NONE) {
//                        motif = motifs.GPP; // XXX for testing, remove.
//                    }
//                })
//                .setInterruptible(false)
//                .requires(this)
//                .setIsDone(() -> motif != motifs.NONE);

        public void scanMotif() {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials.size() >= 2) {
                    motifs detected = getMotif(fiducials);
                    if (detected != motifs.NONE) {
                        motif = detected;
                    }
                }
            }
        }

        public void scanMotifSingle() {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials.size() >= 1) {
                    int tag = fiducials.get(0).getFiducialId();
                    motifs detected = motifs.NONE;
                    if (positions.redAlliance) {
                        if (tag == 23) detected = motifs.PPG;
                        else if (tag == 22) detected = motifs.PGP;
                        else if (tag == 21) detected = motifs.GPP;
                    } else {
                        if (tag == 23) detected = motifs.PPG;
                        else if (tag == 21) detected = motifs.GPP;
                        else if (tag == 22) detected = motifs.PGP;
                    }
                    if (detected != motifs.NONE) motif = detected;
                }
            }
        }

//        public void scanMotif() {
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//                List<Integer> tags = new ArrayList<>();
//                for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                    tags.add(fiducial.getFiducialId());
//                }
//                motifs detected = getMotif(tags);
//                if (detected != motifs.NONE) {
//                    motif = detected;
//                }
//                // No else — keep last known value if tags not currently visible
//            }
//        }
    }

    public static class Index implements Subsystem {
        public static final Index INSTANCE = new Index();

        private Index() {
        }

        // Actuators
        public final ServoEx flicker1 = new ServoEx("flicker1");
        public final ServoEx flicker2 = new ServoEx("flicker2");
        public final ServoEx flicker3 = new ServoEx("flicker3");

        public ServoEx getFlicker1() {
            return flicker1;
        }
        public ServoEx getFlicker2() {
            return flicker2;
        }
        public ServoEx getFlicker3() {
            return flicker3;
        }
        public Command launch1 = new SequentialGroupFixed(
                new SetPosition(flicker1, flicker1up),
                new Delay(0.2),
                new SetPosition(flicker1, flicker1down),
                new Delay(0.12)
        ).setInterruptible(false);

        public Command transfer2 = new SequentialGroupFixed(
                new SetPosition(flicker2, flicker2transfer),
                new Delay(0.1)
        );
        public Command down2 = new SequentialGroupFixed(
                new SetPosition(flicker2, flicker2down),
                new Delay(0.1)
        );
        public Command launch2 = new SequentialGroupFixed(
                new SetPosition(flicker2, flicker2up),
                new Delay(0.2),
                new SetPosition(flicker2, flicker2down),
                new Delay(0.12)
        ).setInterruptible(false);
        public Command launch3 = new SequentialGroupFixed(
                new SetPosition(flicker3, flicker3up),
                new Delay(0.2),
                new SetPosition(flicker3, flicker3down),
                new Delay(0.12)
        ).setInterruptible(false);

        public final Command alldown = new ParallelGroup(
                new SetPosition(flicker1, flicker1down),
                new SetPosition(flicker2, flicker2down),
                new SetPosition(flicker3, flicker3down)
        );


        public Command sensedunsorted;
        boolean launchifballdone = false;

        public Command launchIfBall = new LambdaCommand()
                .setStart(() -> {
                    launchifballdone = false;
                })
                .setUpdate(() -> {
                    if (isoccupied[0]) {
                        launch1.schedule();
                        launchifballdone = true;

                    } else if (isoccupied[1]) {
                        launch2.schedule();
                        launchifballdone = true;

                    } else if (isoccupied[2]) {
                        launch3.schedule();
                        launchifballdone = true;
                    } else {
                        launchifballdone = true;
                    }
                })
                .setIsDone(() -> launchifballdone);


        // Merge these two launches to have one with ifelse
        int count = 0;
        boolean sensingunsorteddone = false;
        String balls = "full";
        private static final long FLICK_COOLDOWN_MS = 300;
        private long lastFlickTimeMs = 0;

        int state = 0;

        public Command closeunsortedlaunch = new SequentialGroupFixed(
                launch1,
                launch2,
                launch3,
                new Delay(0.25)
        ).setInterruptible(false);

        private int index = 0;

        private String slotToString(int zeroBasedIndex) {
            return Integer.toString(zeroBasedIndex + 1);
        }


        public LambdaCommand flickerOrder_disabled = new LambdaCommand()
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
                            flickOrder = "321";
                            break;
                    }

                })
                .setIsDone(() -> true);
        ;
        // Launch 3 in order (based on motif) with a .25sec delay after each and adjust the hood as we go
        // This is best for "close" shooting
        public Command closeSortedLaunch() {
            Command[] order = getFlickOrder();
            double[] hoods = getHoodOrder();
            return new SequentialGroupFixed(
                    order[0],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[0])),
                    new Delay(0.25),
                    order[1],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[1])),
                    new Delay(0.25),
                    order[2],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[2])),
                    new Delay(0.25)
            );
        }

        public Command farSortedLaunch() {
            Command[] order = getFlickOrder();
            double[] hoods = getHoodOrder();
            return new SequentialGroupFixed(
                    new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                    order[0],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[0])),
                    new Delay(0.25),
                    new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                    order[1],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[1])),
                    new Delay(0.25),
                    new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                    order[2],
                    new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[2])),
                    new Delay(0.25)
            );
        }

        private Command[] getFlickOrder() {
            switch (flickOrder) {
                case "132": return new Command[]{launch1, launch3, launch2};
                case "213": return new Command[]{launch2, launch1, launch3};
                case "231": return new Command[]{launch2, launch3, launch1};
                case "312": return new Command[]{launch3, launch1, launch2};
                case "321": return new Command[]{launch3, launch2, launch1};
                default:    return new Command[]{launch1, launch2, launch3};
            }
        }

        private double[] getHoodOrder() {
            switch (flickOrder) {
                case "132": return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
                case "213": return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
                case "231": return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
                case "312": return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
                case "321": return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
                default:    return new double[]{closeHood+0.4, closeHood+0.35, closeHood+0.3};
            }
        }

        public Command farunsortedlaunch = new SequentialGroupFixed(
                launch2,
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                launch1,
                new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                launch3

        );

        // Launch 3, but only if each spot is occupied, and wait until we're atvelocity between shots
        // This is intended for "far" launches, unsorted
        public Command sensedunsortedatspeed() {
            return new SequentialGroupFixed(
                    new IfElseCommand(() -> isoccupied[0], new SequentialGroupFixed(
                            new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                            launch1
                    ), new NullCommand()),
                    new IfElseCommand(() -> isoccupied[1], new SequentialGroupFixed(
                            new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                            launch2
                    ), new NullCommand()),
                    new IfElseCommand(() -> isoccupied[2], new SequentialGroupFixed(
                            new WaitUntil(() -> Thrower.INSTANCE.atvelocity),
                            launch3
                    ), new NullCommand())
            );
        }

        public void initialize() {

        }

        public void periodic() {
            // Always keep flickOrder up to date based on current indexOrder and motif
            if (indexOrder.contains('g') && indexOrder.contains('p')) {
                List<Integer> gSlots = new ArrayList<>();
                List<Integer> pSlots = new ArrayList<>();
                for (int i = 0; i < indexOrder.size(); i++) {
                    if (indexOrder.get(i) == 'g') gSlots.add(i + 1);
                    if (indexOrder.get(i) == 'p') pSlots.add(i + 1);
                }

                if (!gSlots.isEmpty() && !pSlots.isEmpty()) {
                    String g  = Integer.toString(gSlots.get(0));
                    String p1 = Integer.toString(pSlots.get(0));
                    String p2 = pSlots.size() > 1 ? Integer.toString(pSlots.get(1)) : g;

                    switch (motif) {
                        case GPP: flickOrder = g  + p1 + p2; break;
                        case PGP: flickOrder = p1 + g  + p2; break;
                        case PPG: flickOrder = p1 + p2 + g;  break;
                        default:  flickOrder = "123";         break;
                    }
                }
            } else {
                flickOrder = "123";
            }

            // sensedunsorted can stay here too since it was already being rebuilt every loop
            sensedunsorted = new SequentialGroupFixed(
                    new IfElseCommand(() -> isoccupied[0], launch1, new NullCommand()),
                    new IfElseCommand(() -> isoccupied[1], launch2, new NullCommand()),
                    new IfElseCommand(() -> isoccupied[2], launch3, new NullCommand())
            );
        }

    }

    public static class Intake implements Subsystem {
            public static final Intake INSTANCE = new Intake();

            private Intake() {
            }

            public DcMotorEx intakeMotor; // = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
            double intakePower = -1;
            public static boolean negative = false;

            @Override
            public void initialize() {
                Subsystem.super.initialize();
                intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
            }

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

        DcMotorEx thrower1; // = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2; // = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower2");
        Servo hood; // = ActiveOpMode.hardwareMap().get(Servo.class, "hood");

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

            thrower1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower1");
            thrower2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower2");
            hood = ActiveOpMode.hardwareMap().get(Servo.class, "hood");

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

                if (far) targetvelocity = 1600; //1650
                else if (!Double.isNaN(ll.fetchFlywheelSpeed(limelight))) { targetvelocity = ll.fetchFlywheelSpeed(limelight); }
                if (far) hoodpos = 0.08;
                else if (!Double.isNaN(ll.fetchHoodPos(limelight))) { hoodpos = ll.fetchHoodPos(limelight); }
                if (Double.isNaN(hoodpos)) hoodpos = 0.08;

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
//                ActiveOpMode.telemetry().addData("atvelocity: ", atvelocity);
//                ActiveOpMode.telemetry().addData("targetvelocity ", targetvelocity);
//                ActiveOpMode.telemetry().addData("velocity/28*60 ", (thrower1.getVelocity() / 28) * 60);

                // Do not remove this one unless you check with Crocker:
                if (thrower1.getVelocity() == 0 || thrower2.getVelocity() == 0) {
                    ActiveOpMode.telemetry().addLine("ENCODER CABLE UNPLUGGED???");
                }
// rely on later telemetry to do the update() so it's on one screen
//                ActiveOpMode.telemetry().update();

//                TelemetryManager.TelemetryWrapper panelstel = PanelsTelemetry.INSTANCE.getFtcTelemetry();
//
//                panelstel.addData("targetvelocity ", targetvelocity);
//                panelstel.addData("thrower1/28*60", (thrower1.getVelocity() / 28) * 60);
//                panelstel.addData("hoodpos", hoodpos);
//                panelstel.addData("atvelocity", atvelocity);
//                panelstel.addData("error", targetvelocity - (thrower1.getVelocity() / 28) * 60);
//                panelstel.update();


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

        private DigitalChannel magnet;

        public String telemetryWarning = "";

        private Turret() {
        }

        public boolean atposition = false;

        public static DcMotorEx turret; // = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
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

        public Command bluegoalinit = new InstantCommand(() -> {
            turretTargetPos = blueGoalInit;
        });
        public Command bluefarinit = new InstantCommand(() -> {
            turretTargetPos = blueFarInit;
        });
        public Command bluefar = new InstantCommand(() -> {
            turretTargetPos = blueFarPickup;
        });
        public Command bluegoal = new InstantCommand(() -> {
            turretTargetPos = blueGoalPickup;
        });

        public Command bluepark = new InstantCommand(() -> {
            turretTargetPos = blueGoalPark;
        });

        public Command home = new InstantCommand(() -> {
            turretTargetPos = 0;
        });


//        public Command redinit = new LambdaCommand()
//                .setStart(() -> {turretTargetPos = redFarInit;
//                })
//                .setIsDone(() -> Math.abs(Math.abs(turret.getCurrentPosition()) - Math.abs(turretTargetPos)) <= 2);

        public void initialize() {
            telemetryWarning = "";
            // xxx new Thursday -gc
            subsystems.far = false;
            subsystems.motif = subsystems.motifs.NONE;

            Subsystem.super.initialize();
            magnet = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "magnet");
            magnet.setMode(DigitalChannel.Mode.INPUT);
            turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPositionPIDFCoefficients(150);

            if (magnet.getState()) {
                // MAGNET IS NOT SENSED (true=not sensed, oddly) -> WE ARE NOT IN THE RIGHT POSITION!
                telemetryWarning = "WARNING: MAGNET NOT SENSED. CHECK TURRET POSITION.";

//                // Try to rotate slowly until magnet is sensed, but with a limit?
//                // Try to rotate slowly until magnet is sensed, but with a limit?
//                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                turret.setPower(-.6);   // Hope that this is the correct direction
//                while (ActiveOpMode.opModeInInit() && magnet.getState()) {
//                    //
//                }
//                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                // We actually need to go a BIT further so we're "home" but unclear how to
//                turret.setTargetPosition(2);
//                while (ActiveOpMode.opModeInInit() && turret.getCurrentPosition() <= 2) {
//                    //
//                }
//                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Magnet is on. Safe to rotate to starting position:
            turret.setTargetPosition(initPos);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        public void periodic() {
            Subsystem.super.periodic();
            turret.setTargetPosition(turretTargetPos);
            turret.setPower(1);
            atposition = (Math.abs(turret.getCurrentPosition() - turretTargetPos) <= 2);

            if (telemetryWarning.length() > 0) {
                ActiveOpMode.telemetry().addLine(telemetryWarning);
            }
            ActiveOpMode.telemetry().addData("turretTargetPos", turretTargetPos);
            ActiveOpMode.telemetry().addData("turretCurrentPos", turret.getCurrentPosition());
            ActiveOpMode.telemetry().addData("turretError", turretTargetPos - turret.getCurrentPosition());
            ActiveOpMode.telemetry().addData("turretPower", turret.getPower());
            ActiveOpMode.telemetry().addData("turretAtPosition", atposition);
            ActiveOpMode.telemetry().addData("hoodpos", Thrower.hoodpos);
            ActiveOpMode.telemetry().addData("motif", motif);
//            ActiveOpMode.telemetry().addLine("--- COLOR SENSING ---");
            ActiveOpMode.telemetry().addData("isOccupied1", isoccupied[0]);
            ActiveOpMode.telemetry().addData("isOccupied2", isoccupied[1]);
            ActiveOpMode.telemetry().addData("isOccupied3", isoccupied[2]);
            ActiveOpMode.telemetry().addData("indexOrder", indexOrder.toString());
            ActiveOpMode.telemetry().addData("flickOrder", flickOrder);

//            ActiveOpMode.telemetry().addData("swatch[0]", ColorSensing.result1 != null ? ColorSensing.colorToRGBServo(ColorSensing.result1) : "null");
//            ActiveOpMode.telemetry().addData("swatch[1]", ColorSensing.result2 != null ? ColorSensing.colorToRGBServo(ColorSensing.result2) : "null");
//            ActiveOpMode.telemetry().addData("swatch[2]", ColorSensing.result3 != null ? ColorSensing.colorToRGBServo(ColorSensing.result3) : "null");
//            ActiveOpMode.telemetry().addData("flick count", Index.INSTANCE.count);
//            if (magnet.getState()) {
//                ActiveOpMode.telemetry().addLine("no magnet");
//            } else {
//                ActiveOpMode.telemetry().addLine("magnet sensed");
//            }
            ActiveOpMode.telemetry().update();
        }

    }
}
