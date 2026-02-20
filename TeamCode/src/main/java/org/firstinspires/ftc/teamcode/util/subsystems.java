package org.firstinspires.ftc.teamcode.util;

//import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.indexPower;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.ShooterPIDConfig.*;
import static org.firstinspires.ftc.teamcode.util.posConstants.turret;
import static org.firstinspires.ftc.teamcode.util.positions.turretPos;

import android.provider.Settings;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.ArrayList;
import java.util.List;

        import dev.nextftc.core.commands.Command;
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

@Config
public class subsystems {
    public static boolean start = false;

    public static class ColorSensing implements Subsystem {

        public static final ColorSensing INSTANCE = new ColorSensing();

        private ColorSensing() {
        }

        ColorSensor f1s1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f1s1");
        ColorSensor f1s2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f1s2");

        ColorSensor f2s1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f2s1");
        ColorSensor f2s2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f2s2");
        ColorSensor f3s1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f3s1");
        ColorSensor f3s2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "f3s2");

        enum colors {
            P,
            G
        }

        colors flick1color = colors.P;
        colors flick2color = colors.P;
        colors flick3color = colors.P;

        @Override
        public void periodic() {

            if (f1s1.green() > blueThreshold || f1s2.blue() > blueThreshold) {
                flick1color = colors.P;
            } else if (f1s1.green() > greenThreshold || f1s2.green() > greenThreshold) {
                flick1color = colors.G;
            } else {
                flick1color = colors.P;
            }

            if (f2s1.blue() > blueThreshold || f2s2.blue() > blueThreshold) {
                flick2color = colors.P;
            } else if (f2s1.green() > greenThreshold || f2s2.green() > greenThreshold) {
                flick2color = colors.G;
            } else {
                flick2color = colors.P;
            }

            if (f3s1.blue() > blueThreshold || f3s2.blue() > blueThreshold) {
                flick3color = colors.P;
            } else if (f3s1.green() > greenThreshold || f3s2.green() > greenThreshold) {
                flick3color = colors.G;
            } else {
                flick3color = colors.P;
            }
        }

    }

    public static class Camera implements Subsystem {
        public static final Camera INSTANCE = new Camera();

        private Camera() {
        }

        enum motifs {
            PPG,
            PGP,
            GPP,
            NONE
        }

        motifs motif = motifs.NONE;

        motifs getMotif(int tag) {
            if (tag == 21) {
                return motifs.GPP;
            } else if (tag == 22) {
                return motifs.PGP;
            } else if (tag == 23) {
                return motifs.PPG;
            }
            return motifs.NONE;
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
                            motif = getMotif(id);
                            ActiveOpMode.telemetry().addData("motif", motif);
                        }
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
        // Color Sensors


        // State machine
        private int indexState = 0;
        private int indexno = 0;
        private long indexStateTime = 0;
        private boolean launchingWait = false;

        // Config / constants (tweakable)
        public static double autoIndex = -0.1;
        public static double correctIndex = 0.1;
        public static double indexEngaged = 0.84;
        public static double indexDisengaged = 0.7;
        public static double intakePower = 0.5;
        public static int targetCount = 3; // how many launches to perform

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

        // add launch purple command here next time
        private int launchno = 1;
//
//        public LambdaCommand launchmotif = new LambdaCommand()
//                .setUpdate(() -> {
//                    switch(Camera.INSTANCE.motif) {
//                        case PPG:
//                            launch1.schedule();
//                            launchno++;
//                        case PGP:
//
//                    }
//                })
//                .requires(this, Camera.INSTANCE, launch3, launch2, launch1)
//                .setIsDone(() -> launchno > 3);

        private boolean isunsortedlaunchdone = false;

        public Command closeunsortedlaunch = new SequentialGroup(
                launch2,
                new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.4);}),
                new Delay(0.25),
                launch1,
                new Delay(0.25),
                launch3,
                new InstantCommand(() -> {Thrower.INSTANCE.hood.setPosition(closeHood + 0.3);}),
                new Delay(0.25)

        );

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

        private DcMotorEx intake = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
        double intakePower = -1;
        public static boolean negative = true;

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            if (start) {
                intakePower = (negative ? -1 : 0.5);
                intake.setPower(intakePower);
            }
        }
    }

    public static class Thrower implements Subsystem {
        public static final Thrower INSTANCE = new Thrower();

        public static double targetvelocity = 1300;

        private double velocity = 0;

        private Thrower() {
        }

        public boolean atvelocity = false;
        public boolean isshooteron = false;

        DcMotorEx thrower1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower1");
        DcMotorEx thrower2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "thrower2");
        Servo hood = ActiveOpMode.hardwareMap().get(Servo.class, "hood");

        public static PIDFController controller;
        public static Limelight3A limelight;

        public static double hoodpos = .3;

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

            //thrower1.setVelocity(0);
            //thrower2.setPower(thrower1.getPower());
        }

        public void periodic() {
            Subsystem.super.periodic();
            if (start) {

                isshooteron = true;

                targetvelocity = 1400; //1500
                hoodpos = .25;

//                targetvelocity = ll.fetchFlywheelSpeed(limelight);
//                hoodpos = ll.fetchHoodPos(limelight);

                atvelocity = (targetvelocity) - (thrower1.getVelocity()/28)*60 <= 10;

                double currentVelocity = (thrower1.getVelocity()/28)*60;
                double pid = controller.calculate(currentVelocity , targetvelocity);

                if(targetvelocity < 100)
                {
                    thrower1.setMotorDisable();
                    thrower2.setMotorDisable();
                }
                else {
                    if (isshooteron) {
                        thrower1.setPower(pid);
                        thrower2.setPower(pid);
                    }
                }

                hood.setPosition(hoodpos);

                ActiveOpMode.telemetry().addData("num: ", Math.abs(Math.abs(thrower1.getVelocity()) - Math.abs(velocity)));
                ActiveOpMode.telemetry().addData("atvelocity: ", atvelocity);
                ActiveOpMode.telemetry().addData("target ", targetvelocity);
                ActiveOpMode.telemetry().addData("velocity", (thrower1.getVelocity()/28)*60);
                ActiveOpMode.telemetry().update();

                TelemetryManager.TelemetryWrapper panelstel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

                panelstel.addData("target", targetvelocity);
                panelstel.addData("actual", (thrower1.getVelocity()/28)*60);
                panelstel.addData("hood", hoodpos);
                panelstel.addData("atvelocity", atvelocity);
                panelstel.addData("error", targetvelocity - (thrower1.getVelocity()/28)*60);
                panelstel.update();


            } else {
                Index.INSTANCE.alldown.schedule();
                thrower1.setPower(0);
                thrower2.setPower(0);
                velocity = 1300;
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

//            if (Math.abs(turret.getCurrentPosition() - turretTargetPos) <= 1) turret.setPower(0);
//            else if (turret.getCurrentPosition() > turretTargetPos) turret.setPower(-0.075);
//            else turret.setPower(0.075);
//            atposition = (turret.getPower() == 0);
            turret.setTargetPosition(turretTargetPos);
            turret.setPower(1); //turnPower
            atposition = (Math.abs(turret.getCurrentPosition() - turretTargetPos) <= 2);
            ActiveOpMode.telemetry().addData("target:", turretTargetPos);
            ActiveOpMode.telemetry().addData("current:", turret.getCurrentPosition());
            ActiveOpMode.telemetry().addData("power:", turret.getPower());
            ActiveOpMode.telemetry().addLine("TURRET AIM");
            ActiveOpMode.telemetry().update();

        }

    }
}
