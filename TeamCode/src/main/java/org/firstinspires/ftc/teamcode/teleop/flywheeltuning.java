package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp // Registers this OpMode as a TeleOp.
public class flywheeltuning extends OpMode {
    public DcMotorEx thrower1;
    public DcMotorEx thrower2;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    // Initial PIDF coefficients for tuning.
    double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    double P = 265;    // Proportional gain to correct error based on how far off the velocity is.
    double I = 0.01;     // Not sure we really need this but <shrug> -Crocker

    // Array of step sizes for making fine or coarse adjustments to P and F.
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    // Index to select the current step size from the array.
    int stepIndex = 1;


    @Override
    public void init() {
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower1");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0, F);
        thrower1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        thrower2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // --- Gamepad Controls for Tuning ---

        // 'Y' button toggles the target velocity between the high and low presets.
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        // 'B' button cycles through the different step sizes for tuning precision.
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the F (Feedforward) gain.
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the P (Proportional) gain.
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0, F);
        // Apply the new coefficients to the motor in every loop iteration.
        thrower1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        thrower2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Command the motor to run at the current target velocity.
        thrower1.setVelocity(curTargetVelocity);
        thrower2.setVelocity(curTargetVelocity);
//        if (thrower1.getPower() > 0.9) {
//            thrower2.setPower(0.9);
//        } else {
//            thrower2.setPower(thrower1.getPower());
//        }

        // --- Telemetry Output ---

        double curVelocity = thrower1.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("thrower1.getVelocity()", "%.4f", thrower1.getVelocity());
        telemetry.addData("thrower2.getVelocity()", "%.4f", thrower2.getVelocity());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", curTargetVelocity);
        packet.put("Current Velocity", curVelocity);
        packet.put("baseline", 0);
        dashboard.sendTelemetryPacket(packet);
    }
}