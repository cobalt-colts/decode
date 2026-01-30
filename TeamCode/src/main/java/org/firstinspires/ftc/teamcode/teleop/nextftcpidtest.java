package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ShooterPIDConfig;
import org.firstinspires.ftc.teamcode.util.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@Configurable
@TeleOp(name = "NextFTC Shooter PID Test")
public class nextftcpidtest extends NextFTCOpMode {

    public nextftcpidtest() throws InterruptedException {
        addComponents(
                new SubsystemComponent(Thrower.INSTANCE)
        );
    }

    public static PIDCoefficients coefficients = new PIDCoefficients(ShooterPIDConfig.kP, ShooterPIDConfig.kI, ShooterPIDConfig.kD);
    public static BasicFeedforwardParameters ff = new BasicFeedforwardParameters(0,0,0);
    public static double goal = 1000;

    private static double velocity;


    public static class Thrower implements Subsystem {
        public static final Thrower INSTANCE = new Thrower();

        private Thrower() { }

        private final MotorEx thrower1 = new MotorEx("thrower1");
        private final MotorEx thrower2 = new MotorEx("thrower2");

        private final ControlSystem controlSystem = ControlSystem.builder()
                .velPid(coefficients)
                .basicFF(ff)
                .build();

        public Command setspeed = new RunToVelocity(controlSystem, goal);

        @Override
        public void periodic() {
            Subsystem.super.periodic();
            double power = controlSystem.calculate(new KineticState(thrower1.getCurrentPosition(), thrower1.getVelocity()));
            velocity = thrower1.getVelocity();
            thrower1.setPower(-power);
            thrower2.setPower(-power);
        }
    }

    @Override public void onUpdate() {
        Thrower.INSTANCE.setspeed.schedule();
        telemetry.addData("velocity", velocity);
        telemetry.addLine("success!");
    }


}
