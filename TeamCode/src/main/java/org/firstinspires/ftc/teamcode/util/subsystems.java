package org.firstinspires.ftc.teamcode.util;

import android.service.controls.Control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.stream.DoubleStream;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class subsystems {

    private static void index(){
        return;
    }
    public static class Indexer implements Subsystem {
        public static final Indexer INSTANCE = new Indexer();
        private Indexer() { }

        private CRServoEx indexer = new CRServoEx("indexer");
        private ServoEx indexengage = new ServoEx("indexEngage");

        public Command engageindex = new SetPosition(indexengage, 0.825);
        public Command disengageindex = new SetPosition(indexengage, 0.7);

        private ControlSystem controlSystem = ControlSystem.builder()
                .build();
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

        private MotorEx thrower1 = new MotorEx("thrower1");
        private MotorEx thrower2 = new MotorEx("thrower2");

        private ControlSystem controlSystem = ControlSystem.builder()
                .posPid(100, 0, 0)
                .basicFF(0.000357)
                .build();

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            double power = controlSystem.calculate(thrower1.getState());
            thrower1.setPower(power);
            thrower2.setPower();
        }

    }
}
