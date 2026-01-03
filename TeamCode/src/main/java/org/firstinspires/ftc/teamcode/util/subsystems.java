package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.indexPower;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;

import android.graphics.Color;
import android.service.controls.Control;

import com.google.gson.internal.bind.SqlDateTypeAdapter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

import java.util.List;
import java.util.Set;
import java.util.stream.DoubleStream;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import kotlin.jvm.internal.Lambda;

public class subsystems {

    public static class ColorSensing implements Subsystem {

        public static final ColorSensing INSTANCE = new ColorSensing();

        private ColorSensing() {}

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
        public void periodic(){

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

        private Camera() {}

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
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId(); // The ID number of the fiducial
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

        private Index() {}

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

        // Commands to control the engage servo from other code
        public SequentialGroup launch1 = new SequentialGroup(
                new SetPosition(flicker1, lift1Up),
                new Delay(250),
                new SetPosition(flicker1, lift1Down)
        );
        public SequentialGroup launch2 = new SequentialGroup(
                new SetPosition(flicker2, lift2Up),
                new Delay(250),
                new SetPosition(flicker2, lift2Down)
        );
        public SequentialGroup launch3 = new SequentialGroup(
                new SetPosition(flicker3, lift3Up),
                new Delay(250),
                new SetPosition(flicker3, lift3Down)
        );

        public ParallelGroup alldown = new ParallelGroup(
                new SetPosition(flicker1, lift1Down),
                new SetPosition(flicker2, lift2Down),
                new SetPosition(flicker3, lift3Down)
        );

        // add launch purple command here next time
        private int launchno = 1;

        public LambdaCommand launchmotif = new LambdaCommand()
                .setUpdate(() -> {
                    switch(Camera.INSTANCE.motif) {
                        case PPG:
                            launch1.schedule();
                            launchno++;
                        case PGP:

                    }
                })
                .requires(this, Camera.INSTANCE, launch3, launch2, launch1)
                .setIsDone(() -> launchno > 3);


    }
    public static class Intake implements Subsystem {
            public static final Intake INSTANCE = new Intake();

            private Intake(){ }

            private MotorEx intake = new MotorEx("intake");

            public Command intakeon = new SetPower(intake,-1).requires(this);
            public Command intakeoff = new SetPower(intake, 0).requires(this);
    }
    public static class Thrower implements Subsystem {
        public static final Thrower INSTANCE = new Thrower();

        private Thrower() { }

        private final MotorEx thrower1 = new MotorEx("thrower1");
        private final MotorEx thrower2 = new MotorEx("thrower2");

        private final ControlSystem controlSystem = ControlSystem.builder()
                .velPid(100, 0, 0)
                .armFF(0.000357)
                .build();

        public Command spinup = new LambdaCommand()
                .setStart(() -> {
                    new RunToVelocity(controlSystem, 800).schedule();
                        }
                )
                .requires(this)
                .setInterruptible(true)
                .setIsDone(() -> true)
                .named("Spin Thrower Up");

        public Command autoshootpos = new LambdaCommand()
                .setStart(() -> {
                    new RunToVelocity(controlSystem, 400, 25).schedule();
                        }
                )
                .requires(this)
                .setInterruptible(true)
                .setIsDone(() -> true)
                .named("Spin auto thrower");

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            double power = controlSystem.calculate(thrower1.getState());
            thrower1.setPower(power);
            thrower2.setPower(power);
        }
    }
}
