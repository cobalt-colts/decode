package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.indexPower;
import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.lift1Down;
import static org.firstinspires.ftc.teamcode.teleop.meet2teleop.lift1Up;

import android.service.controls.Control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

import java.util.Set;
import java.util.stream.DoubleStream;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
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

    public static class Index implements Subsystem {
        public static final Index INSTANCE = new Index();

        private Index() {}

        // Actuators
        private final ServoEx indexengage = new ServoEx("indexengage");
        private final CRServoEx indexer = new CRServoEx("indexer");
        private final MotorEx intake = new MotorEx("intake");
        private final ServoEx lift = new ServoEx("lift");

        // Sensors (raw hardware channels)
        DigitalChannel magnet1 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "mag1");
        DigitalChannel magnet2 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "mag2");
        DigitalChannel magnet3 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "mag3");

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
        public Command engage = new SetPosition(indexengage, indexEngaged);
        public Command disengage = new SetPosition(indexengage, indexDisengaged);

        // Non-blocking index routine implemented as a LambdaCommand
        public LambdaCommand index3 = new LambdaCommand()
                .setStart(() -> {
                    indexState = 0;
                    indexno = 0;
                    indexStateTime = System.currentTimeMillis();
                    launchingWait = false;
                })
                .setUpdate(() -> {
                    // read current (logical) magnet states: true == magnet present on prong
                    boolean mag1State = !magnet1.getState();
                    boolean mag2State = !magnet2.getState();
                    boolean mag3State = !magnet3.getState();

                    long now = System.currentTimeMillis();

                    switch (indexState) {
                        // START: engage, run intake + indexer
                        case 0:
                            INSTANCE.engage.schedule();
                            new SetPower(intake, intakePower);
                            new SetPower(indexer, autoIndex);

                            // If any magnet is present at the start, immediately perform a launch
                            if (mag1State || mag2State || mag3State) {
                                Lift.INSTANCE.liftup.schedule();// uses static import from meet2teleop
                                indexStateTime = now;
                                launchingWait = true;
                                indexState = 1; // go to indexing/launch wait state
                                break;
                            }

                            // otherwise move to normal indexing state
                            indexState = 1;
                            indexStateTime = now;
                            break;

                        // INDEXING: run the indexer and trigger launches when the alignment condition is met
                        case 1:
                            // If we were doing an immediate launch wait, finish the wait then drop the lift
                            if (launchingWait) {
                                if (now - indexStateTime >= 200) {
                                    Lift.INSTANCE.liftdown.schedule();
                                    launchingWait = false;
                                    indexno++;
                                    // if we've completed targetCount launches, stop and finish
                                    if (indexno >= targetCount) {
                                        indexer.setPower(0);
                                        intake.setPower(0);
                                        indexState = 67;
                                    }
                                }
                                break;
                            }

                            // Normal indexing: simple rule — when mag3 & mag1 are simultaneously present,
                            // consider that an aligned ball for launch (mirrors ctREDGoal behavior)
                            if (mag3State && mag1State) {
                                // momentarily stop advancing and actuate the lift to launch
                                new SetPower(indexer, 0);
                                Lift.INSTANCE.liftup.schedule();
                                // schedule a short wait using timestamping rather than blocking
                                indexStateTime = now;
                                launchingWait = true;
                                // keep state 1 to process the wait and post-launch behavior
                                break;
                            }

                            // otherwise keep the indexer moving at autoIndex
                            new SetPower(indexer, autoIndex);
                            break;

                        // CLEANUP / finished one ball (not heavily used in this simplified flow)
                        case 2:
                            new SetPower(indexer, 0);
                            new SetPower(intake, 0);
                            if (indexno >= targetCount) {
                                indexState = 67;
                            } else {
                                indexState = 0;
                            }
                            break;

                        default:
                            break;
                    }
                })
                .setIsDone(() -> indexState == 67)
                .setInterruptible(true);

    }
    public static class Intake implements Subsystem {
            public static final Intake INSTANCE = new Intake();

            private Intake(){ }

            private MotorEx intake = new MotorEx("intake");

            public Command intakeon = new SetPower(intake,-0.4).requires(this);
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
    public static class Lift implements Subsystem {
        public static final Lift INSTANCE = new Lift();

        private Lift() { }

        private ServoEx lift = new ServoEx("lift");

        public Command liftup = new SetPosition(lift, 0.5);
        public Command liftdown = new SetPosition(lift, 0.25);


    }
}
