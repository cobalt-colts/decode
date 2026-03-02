package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.posConstants.limelight;
import static org.firstinspires.ftc.teamcode.util.posConstants.limelightFast;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.posConstants;

import java.util.List;

//@Configurable
@Config
@TeleOp(name="Limelight Test")
public class LimelightTest extends LinearOpMode {

    double ta, ty, yaw;

    @Override
    public void runOpMode() throws InterruptedException {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(limelightFast);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            LLResultTypes.FiducialResult tag = tags.get(0);
            ta = result.getTa();
            ty = result.getTy();
            yaw = tag.getCameraPoseTargetSpace().getOrientation().getYaw();

            telemetry.addData("getTa", ta);
            telemetry.addData("getTy", ty);
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }
}