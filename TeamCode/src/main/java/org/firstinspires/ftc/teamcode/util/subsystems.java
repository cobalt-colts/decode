package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.service.controls.Control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

import java.util.stream.DoubleStream;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.subsystems.Subsystem;
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

        private ServoEx indexengage = new ServoEx("indexengage");

        public Command engage = new SetPosition(indexengage, .84);
        public Command disengage = new SetPosition(indexengage, .7);
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
