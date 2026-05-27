package org.firstinspires.ftc.teamcode.util.solvers;

import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.kD;
import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.kF;
import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.kI;
import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.kP;
import static org.firstinspires.ftc.teamcode.util.posConstants.blueFarInit;
import static org.firstinspires.ftc.teamcode.util.posConstants.blueFarPickup;
import static org.firstinspires.ftc.teamcode.util.posConstants.blueGoalInit;
import static org.firstinspires.ftc.teamcode.util.posConstants.blueGoalPark;
import static org.firstinspires.ftc.teamcode.util.posConstants.blueGoalPickup;
import static org.firstinspires.ftc.teamcode.util.posConstants.closeHood;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker1up;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker2down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker2up;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker3down;
import static org.firstinspires.ftc.teamcode.util.posConstants.flicker3up;
import static org.firstinspires.ftc.teamcode.util.posConstants.redFarInit;
import static org.firstinspires.ftc.teamcode.util.posConstants.redFarPickup;
import static org.firstinspires.ftc.teamcode.util.posConstants.redGoalInit;
import static org.firstinspires.ftc.teamcode.util.posConstants.redGoalPark;
import static org.firstinspires.ftc.teamcode.util.posConstants.redGoalPickup;
import static org.firstinspires.ftc.teamcode.util.posConstants.turretMax;
import static org.firstinspires.ftc.teamcode.util.posConstants.turretMin;
import static org.firstinspires.ftc.teamcode.util.positions.GREY_SATURATION1;
import static org.firstinspires.ftc.teamcode.util.positions.GREY_SATURATION2;
import static org.firstinspires.ftc.teamcode.util.positions.GREY_SATURATION3;
import static org.firstinspires.ftc.teamcode.util.positions.colorToRGBServo;
import static org.firstinspires.ftc.teamcode.util.positions.isball;
import static org.firstinspires.ftc.teamcode.util.positions.redAlliance;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ll;
import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

public final class subsystems {

    private subsystems() {
    }

    public static boolean start = false;
    public static boolean teleop = false;
    public static boolean far = false;

    public static final char[] motifOrder = {'b', 'b', 'b'};
    public static final ArrayList<Character> indexOrder = new ArrayList<>(3);
    public static int count = 0;
    public static int matchingSpot = -1;
    public static boolean hasAnyBalls = false;
    public static final boolean[] isoccupied = new boolean[3];
    public static String flickOrder = "321";

    public enum motifs {
        PPG,
        PGP,
        GPP,
        NONE
    }

    public static motifs motif = motifs.NONE;

    public static class Thrower extends SubsystemBase {
        public static Thrower INSTANCE;

        public static Limelight3A limelight;

        public static double targetvelocity = 1370;
        public static double hoodpos = 0.3;
        public static boolean atvelocity = false;

        public static double DEFAULT_SHOOT_TA = 0.724;

        private final DcMotorEx thrower1;
        private final DcMotorEx thrower2;
        public final Servo hood;
        private final PIDFController controller;
        private final InterpLUT shootlut;
        private final InterpLUT hoodlut;

        private boolean shooterOn = false;
        private double shootTa = DEFAULT_SHOOT_TA;

        public Thrower(HardwareMap hMap) {
            INSTANCE = this;

            controller = new PIDFController(kP, kI, kD, kF);
            shootlut = new InterpLUT();
            hoodlut = new InterpLUT();

            limelight = hMap.get(Limelight3A.class, "limelight");
            limelight.start();

            thrower1 = hMap.get(DcMotorEx.class, "thrower1");
            thrower2 = hMap.get(DcMotorEx.class, "thrower2");
            hood = hMap.get(Servo.class, "hood");

            thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thrower1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            thrower1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            thrower2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shootlut.add(0.1, 911);
            shootlut.add(0.322, 910);
            shootlut.add(0.724, 775);
            shootlut.add(1.079, 700);
            shootlut.add(1.522, 650);
            shootlut.add(3.09, 600);
            shootlut.add(15, 599);
            shootlut.createLUT();

            hoodlut.add(0.1, .324);
            hoodlut.add(0.322, .325);
            hoodlut.add(0.724, .435);
            hoodlut.add(1.079, .475);
            hoodlut.add(1.522, .5);
            hoodlut.add(3.09, .6);
            hoodlut.add(15, .601);
            hoodlut.createLUT();

            hood.setPosition(0.5);
        }

        public Command shooterOnCommand = new InstantCommand(() ->
                shooterOn = true,
                this
        );

        public Command shooterOffCommand = new InstantCommand(() ->
                shooterOn = false,
                this
        );

        @Override
        public void periodic() {
            if (!start) {
                shooterOn = false;
                thrower1.setPower(0);
                thrower2.setPower(0);
                return;
            }

            shooterOn = true;

            double detectedTa = ll.fetchTa(limelight);
            if (!Double.isNaN(detectedTa) && detectedTa > 0) {
                shootTa = detectedTa;
            }
            targetvelocity = shootlut.get(shootTa);
            hoodpos = hoodlut.get(shootTa);

            double currentVelocity = (thrower1.getVelocity() / 28.0) * 60.0;
            atvelocity = targetvelocity - currentVelocity <= 10;

            if (targetvelocity < 100) {
                thrower1.setMotorDisable();
                thrower2.setMotorDisable();
            } else if (shooterOn) {
                double pid = controller.calculate(currentVelocity, targetvelocity);
                thrower1.setPower(pid);
                thrower2.setPower(pid);
            }

            hood.setPosition(Math.max(0, Math.min(1, hoodpos)));
        }
    }

    public static class Turret extends SubsystemBase {
        public static Turret INSTANCE;

        public static int turretTargetPos = 0;
        public static int initPos = redFarInit;

        private static final double AUTOAIM_GAIN = 1.6;
        private static final int AUTOAIM_MAX_STEP_TICKS = 75;
        private static final int AUTOAIM_MIN_STEP_TICKS = 4;

        public final DcMotorEx turret;
        private final DigitalChannel magnet;

        public String telemetryWarning = "";
        public boolean atposition = false;

        public Turret(HardwareMap hMap) {
            INSTANCE = this;

            magnet = hMap.get(DigitalChannel.class, "magnet");
            magnet.setMode(DigitalChannel.Mode.INPUT);

            turret = hMap.get(DcMotorEx.class, "turret");
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPositionPIDFCoefficients(150);

            if (magnet.getState()) {
                telemetryWarning = "WARNING: MAGNET NOT SENSED. CHECK TURRET POSITION.";
                turret.setTargetPosition(initPos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        public Command redgoalinit = new InstantCommand(() -> turretTargetPos = redGoalInit, this);
        public Command redfarinit = new InstantCommand(() -> turretTargetPos = redFarInit, this);
        public Command redfar = new InstantCommand(() -> turretTargetPos = redFarPickup, this);
        public Command redgoal = new InstantCommand(() -> turretTargetPos = redGoalPickup, this);
        public Command redpark = new InstantCommand(() -> turretTargetPos = redGoalPark, this);

        public Command bluegoalinit = new InstantCommand(() -> turretTargetPos = blueGoalInit, this);
        public Command bluefarinit = new InstantCommand(() -> turretTargetPos = blueFarInit, this);
        public Command bluefar = new InstantCommand(() -> turretTargetPos = blueFarPickup, this);
        public Command bluegoal = new InstantCommand(() -> turretTargetPos = blueGoalPickup, this);
        public Command bluepark = new InstantCommand(() -> turretTargetPos = blueGoalPark, this);

        public Command home = new InstantCommand(() -> turretTargetPos = 0, this);

        @Override
        public void periodic() {
            if (start && teleop && Thrower.limelight != null) {
                double alignmentTicks = ll.fetchAlignment(Thrower.limelight);
                if (!Double.isNaN(alignmentTicks)) {
                    int correctionTicks = (int) Math.round(alignmentTicks * AUTOAIM_GAIN);
                    correctionTicks = Math.max(-AUTOAIM_MAX_STEP_TICKS,
                            Math.min(AUTOAIM_MAX_STEP_TICKS, correctionTicks));

                    if (correctionTicks > 0) {
                        correctionTicks = Math.max(correctionTicks, AUTOAIM_MIN_STEP_TICKS);
                    } else if (correctionTicks < 0) {
                        correctionTicks = Math.min(correctionTicks, -AUTOAIM_MIN_STEP_TICKS);
                    }

                    turretTargetPos = turret.getCurrentPosition() + correctionTicks;
                }
            }

            if (teleop) {
                turretTargetPos = Math.max(turretMin, Math.min(turretMax, turretTargetPos));
            }
            turret.setTargetPosition(turretTargetPos);
            turret.setPower(1);
            atposition = Math.abs(turret.getCurrentPosition() - turretTargetPos) <= 2;
        }
    }

    public static class ColorSensing extends SubsystemBase {
        public static ColorSensing INSTANCE;

        public static PredominantColorProcessor.Result result1;
        public static PredominantColorProcessor.Result result2;
        public static PredominantColorProcessor.Result result3;

        private final Servo light1;
        private final Servo light2;
        private final Servo light3;

        private final PredominantColorProcessor sensor1;
        private final PredominantColorProcessor sensor2;
        private final PredominantColorProcessor sensor3;

        public final VisionPortal portal;

        private long lastToggleTime = 0;
        private boolean lightsOn = false;
        private static final long BLINK_INTERVAL_MS = 250;

        private static final double ALPHA = 0.15;
        private final float[] satAvg = new float[3];
        private final boolean[] initialized = new boolean[3];

        public ColorSensing(HardwareMap hMap) {
            INSTANCE = this;

            light1 = hMap.get(Servo.class, "light1");
            light2 = hMap.get(Servo.class, "light2");
            light3 = hMap.get(Servo.class, "light3");

            sensor2 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, -0.3, -0.3, -0.8))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();

            sensor1 = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(0.4, 0.3, 0.9, -0.3))
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
                    .setCamera(hMap.get(WebcamName.class, "internalcam"))
                    .build();
        }

        private void updateOccupied(int i, PredominantColorProcessor.Result result, long satCutoff) {
            if (result == null) {
                return;
            }

            float sat = result.HSV[1];

            if (!initialized[i]) {
                satAvg[i] = sat;
                initialized[i] = true;
            } else {
                satAvg[i] = (float) (ALPHA * sat + (1 - ALPHA) * satAvg[i]);
            }

            if (isoccupied[i]) {
                if (satAvg[i] < satCutoff - 15) {
                    isoccupied[i] = false;
                }
            } else if (satAvg[i] > satCutoff + 15) {
                isoccupied[i] = true;
            }
        }

        @Override
        public void periodic() {
            result1 = sensor1.getAnalysis();
            result2 = sensor2.getAnalysis();
            result3 = sensor3.getAnalysis();

            updateOccupied(0, result1, GREY_SATURATION1);
            updateOccupied(1, result2, GREY_SATURATION2);
            updateOccupied(2, result3, GREY_SATURATION3);

            indexOrder.clear();
            PredominantColorProcessor.Result[] results = {result1, result2, result3};
            long[] satCutoffs = {GREY_SATURATION1, GREY_SATURATION2, GREY_SATURATION3};

            for (int i = 0; i < 3; i++) {
                PredominantColorProcessor.Result result = results[i];
                if (!isoccupied[i]) {
                    indexOrder.add('b');
                } else if (result != null
                        && result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN
                        && isball(result, satCutoffs[i])) {
                    indexOrder.add('g');
                } else {
                    indexOrder.add('p');
                }
            }

            hasAnyBalls = isoccupied[0] || isoccupied[1] || isoccupied[2];

            if (start && !teleop && Intake.INSTANCE != null) {
                Intake.INSTANCE.intakeMotor.setPower((isoccupied[0] && isoccupied[1] && isoccupied[2]) ? 0.5 : -1);
            }

            long now = System.nanoTime() / 1_000_000;
            if (now - lastToggleTime >= BLINK_INTERVAL_MS) {
                lightsOn = !lightsOn;
                lastToggleTime = now;

                if (lightsOn) {
                    light1.setPosition(colorToRGBServo(result1, GREY_SATURATION1));
                    light2.setPosition(colorToRGBServo(result2, GREY_SATURATION2));
                    light3.setPosition(colorToRGBServo(result3, GREY_SATURATION3));
                }
            }
        }
    }

    public static class Camera extends SubsystemBase {
        public static Camera INSTANCE;

        public final Limelight3A limelight;

        public Camera(HardwareMap hMap) {
            INSTANCE = this;

            limelight = hMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            if (!teleop) {
                limelight.pipelineSwitch(1);
            }
            limelight.start();
        }

        private motifs getMotif(List<LLResultTypes.FiducialResult> fiducials) {
            if (fiducials.size() < 2) {
                return motifs.NONE;
            }

            fiducials.sort((a, b) -> Double.compare(a.getTargetXDegrees(), b.getTargetXDegrees()));
            int leftTag = fiducials.get(0).getFiducialId();
            int rightTag = fiducials.get(1).getFiducialId();

            if (redAlliance) {
                if (leftTag == 23 && rightTag == 22) {
                    return motifs.PPG;
                }
                if (leftTag == 22 && rightTag == 21) {
                    return motifs.PGP;
                }
                if (leftTag == 21 && rightTag == 23) {
                    return motifs.GPP;
                }
            } else {
                if (leftTag == 21 && rightTag == 23) {
                    return motifs.PPG;
                }
                if (leftTag == 22 && rightTag == 21) {
                    return motifs.GPP;
                }
                if (leftTag == 23 && rightTag == 22) {
                    return motifs.PGP;
                }
            }

            return motifs.NONE;
        }

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
                    if (redAlliance) {
                        if (tag == 23) {
                            detected = motifs.PPG;
                        } else if (tag == 22) {
                            detected = motifs.PGP;
                        } else if (tag == 21) {
                            detected = motifs.GPP;
                        }
                    } else {
                        if (tag == 23) {
                            detected = motifs.PPG;
                        } else if (tag == 21) {
                            detected = motifs.GPP;
                        } else if (tag == 22) {
                            detected = motifs.PGP;
                        }
                    }
                    if (detected != motifs.NONE) {
                        motif = detected;
                    }
                }
            }
        }
    }

    public static class Index extends SubsystemBase {
        public static Index INSTANCE;

        private final Servo flicker1;
        private final Servo flicker2;
        private final Servo flicker3;

        public final Command launch1;
        public final Command launch2;
        public final Command launch3;
        public final Command closeunsortedlaunch;
        public final Command alldown;

        public final Command sensedunsorted;
        public final Command sensedunsortedatspeed;

        public Index(HardwareMap hMap) {
            INSTANCE = this;

            flicker1 = hMap.get(Servo.class, "flicker1");
            flicker2 = hMap.get(Servo.class, "flicker2");
            flicker3 = hMap.get(Servo.class, "flicker3");

            launch1 = buildLaunch(flicker1, flicker1up, flicker1down);
            launch2 = buildLaunch(flicker2, flicker2up, flicker2down);
            launch3 = buildLaunch(flicker3, flicker3up, flicker3down);

            alldown = new SequentialCommandGroup(
                    setPosition(flicker1, flicker1down),
                    setPosition(flicker2, flicker2down),
                    setPosition(flicker3, flicker3down)
            );

            closeunsortedlaunch = new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            buildLaunch(flicker3, flicker3up, flicker3down),
                            new WaitCommand(250)
                    )
            );

            sensedunsorted = new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            new ConditionalCommand(buildLaunch(flicker1, flicker1up, flicker1down), new InstantCommand(), () -> isoccupied[0]),
                            new ConditionalCommand(buildLaunch(flicker2, flicker2up, flicker2down), new InstantCommand(), () -> isoccupied[1]),
                            new ConditionalCommand(buildLaunch(flicker3, flicker3up, flicker3down), new InstantCommand(), () -> isoccupied[2])
                    )
            );

            sensedunsortedatspeed = new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            new ConditionalCommand(
                                    new SequentialCommandGroup(new WaitUntilCommand(() -> Thrower.atvelocity), buildLaunch(flicker1, flicker1up, flicker1down)),
                                    new InstantCommand(),
                                    () -> isoccupied[0]
                            ),
                            new ConditionalCommand(
                                    new SequentialCommandGroup(new WaitUntilCommand(() -> Thrower.atvelocity), buildLaunch(flicker2, flicker2up, flicker2down)),
                                    new InstantCommand(),
                                    () -> isoccupied[1]
                            ),
                            new ConditionalCommand(
                                    new SequentialCommandGroup(new WaitUntilCommand(() -> Thrower.atvelocity), buildLaunch(flicker3, flicker3up, flicker3down)),
                                    new InstantCommand(),
                                    () -> isoccupied[2]
                            )
                    )
            );
        }

        private Command setPosition(Servo servo, double pos) {
            return new InstantCommand(() -> servo.setPosition(pos), this);
        }

        private Command buildLaunch(Servo servo, double up, double down) {
            return new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> servo.setPosition(up), this),
                            new WaitCommand(200),
                            new InstantCommand(() -> servo.setPosition(down), this),
                            new WaitCommand(120)
                    )
            );
        }

        private Command[] getFlickOrder() {
            switch (flickOrder) {
                case "132":
                    return new Command[]{
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            buildLaunch(flicker3, flicker3up, flicker3down),
                            buildLaunch(flicker2, flicker2up, flicker2down)};
                case "213":
                    return new Command[]{
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            buildLaunch(flicker3, flicker3up, flicker3down)};
                case "231":
                    return new Command[]{
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            buildLaunch(flicker3, flicker3up, flicker3down),
                            buildLaunch(flicker1, flicker1up, flicker1down)};
                case "312":
                    return new Command[]{
                            buildLaunch(flicker3, flicker3up, flicker3down),
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            buildLaunch(flicker2, flicker2up, flicker2down)};
                case "321":
                    return new Command[]{
                            buildLaunch(flicker3, flicker3up, flicker3down),
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            buildLaunch(flicker1, flicker1up, flicker1down)};
                default:
                    return new Command[]{
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            buildLaunch(flicker3, flicker3up, flicker3down)};
            }
        }

        private double[] getHoodOrder() {
            return new double[]{closeHood + 0.4, closeHood + 0.35, closeHood + 0.3};
        }

        public Command closeSortedLaunch() {
            Command[] order = getFlickOrder();
            double[] hoods = getHoodOrder();
            return new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            order[0],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[0])),
                            new WaitCommand(250),
                            order[1],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[1])),
                            new WaitCommand(250),
                            order[2],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[2])),
                            new WaitCommand(250)
                    )
            );
        }

        public Command farSortedLaunch() {
            Command[] order = getFlickOrder();
            double[] hoods = getHoodOrder();
            return new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            order[0],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[0])),
                            new WaitCommand(250),
                            new WaitUntilCommand(() -> Thrower.atvelocity),
                            order[1],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[1])),
                            new WaitCommand(250),
                            new WaitUntilCommand(() -> Thrower.atvelocity),
                            order[2],
                            new InstantCommand(() -> Thrower.INSTANCE.hood.setPosition(hoods[2])),
                            new WaitCommand(250)
                    )
            );
        }

        public Command farunsortedlaunch() {
            return new UninterruptibleCommand(
                    new SequentialCommandGroup(
                            buildLaunch(flicker2, flicker2up, flicker2down),
                            new WaitUntilCommand(() -> Thrower.atvelocity),
                            buildLaunch(flicker1, flicker1up, flicker1down),
                            new WaitUntilCommand(() -> Thrower.atvelocity),
                            buildLaunch(flicker3, flicker3up, flicker3down)
                    )
            );
        }

        @Override
        public void periodic() {
            if (indexOrder.contains('g') && indexOrder.contains('p')) {
                List<Integer> gSlots = new ArrayList<>();
                List<Integer> pSlots = new ArrayList<>();
                for (int i = 0; i < indexOrder.size(); i++) {
                    if (indexOrder.get(i) == 'g') {
                        gSlots.add(i + 1);
                    }
                    if (indexOrder.get(i) == 'p') {
                        pSlots.add(i + 1);
                    }
                }

                if (!gSlots.isEmpty() && !pSlots.isEmpty()) {
                    String g = Integer.toString(gSlots.get(0));
                    String p1 = Integer.toString(pSlots.get(0));
                    String p2 = pSlots.size() > 1 ? Integer.toString(pSlots.get(1)) : g;

                    switch (motif) {
                        case GPP:
                            flickOrder = g + p1 + p2;
                            break;
                        case PGP:
                            flickOrder = p1 + g + p2;
                            break;
                        case PPG:
                            flickOrder = p1 + p2 + g;
                            break;
                        default:
                            flickOrder = "123";
                            break;
                    }
                }
            } else {
                flickOrder = "123";
            }
        }
    }

    public static class Intake extends SubsystemBase {
        public static Intake INSTANCE;

        public final DcMotorEx intakeMotor;

        public Intake(HardwareMap hMap) {
            INSTANCE = this;
            intakeMotor = hMap.get(DcMotorEx.class, "intake");
            intakeMotor.setPower(0);
        }

        public Command intakeCommand() {
            return new InstantCommand(() -> intakeMotor.setPower(-1), this);
        }

        public Command outtakeCommand() {
            return new InstantCommand(() -> intakeMotor.setPower(0.5), this);
        }

        @Override
        public void periodic() {
            if (!start) {
                intakeMotor.setPower(0);
            }
        }
    }

    public static class Drive extends SubsystemBase {
        public static Drive INSTANCE;

        private final DcMotor frontLeft;
        private final DcMotor frontRight;
        private final DcMotor backLeft;
        private final DcMotor backRight;

        public Drive(HardwareMap hMap) {
            INSTANCE = this;

            frontLeft = hMap.dcMotor.get("frontLeft");
            backLeft = hMap.dcMotor.get("backLeft");
            frontRight = hMap.dcMotor.get("frontRight");
            backRight = hMap.dcMotor.get("backRight");

            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public Command driverControl(GamepadEx driver) {
            return new RunCommand(() -> {
                double y = -driver.getLeftY();
                double x = driver.getLeftX();
                double rx = driver.getRightX();

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                double fl = (y + x + rx) / denominator;
                double bl = (y - x + rx) / denominator;
                double fr = (y - x - rx) / denominator;
                double br = (y + x - rx) / denominator;

                frontLeft.setPower(fl);
                backLeft.setPower(bl);
                frontRight.setPower(fr);
                backRight.setPower(br);
            }, this);
        }

        public Command stopDrive() {
            return new InstantCommand(() -> {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }, this);
        }

        public void stop() {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }

    public static class Robot extends com.seattlesolvers.solverslib.command.Robot {
        public final Thrower thrower;
        public final Turret turret;
        public final Index index;
        public final Intake intake;
        public final Camera camera;
        public final ColorSensing colorSensing;
        public final Drive drive;

        public Robot(HardwareMap hardwareMap) {
            thrower = new Thrower(hardwareMap);
            turret = new Turret(hardwareMap);
            index = new Index(hardwareMap);
            intake = new Intake(hardwareMap);
            camera = new Camera(hardwareMap);
            colorSensing = new ColorSensing(hardwareMap);
            drive = new Drive(hardwareMap);

            register(thrower, turret, index, intake, camera, colorSensing, drive);
        }
    }
}
