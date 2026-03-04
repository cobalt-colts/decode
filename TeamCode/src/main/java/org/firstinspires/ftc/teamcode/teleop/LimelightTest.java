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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.posConstants;

import java.util.List;

//@Configurable
@Config
@TeleOp(name="Limelight Test")
public class LimelightTest extends LinearOpMode {

    double ta, ty, yaw, rightDelta, leftDelta;
    String family = " ";
    LLResultTypes.FiducialResult tag;
    Pose3D space;
    YawPitchRollAngles orientation;

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
//            if (!tags.isEmpty()) tag = tags.get(0);
            ta = result.getTa();
            ty = result.getTy();
            family = tag.getFamily();
////            space = tag.getTargetPoseCameraSpace();
////            orientation = space.getOrientation();
////            yaw = orientation.getYaw();
////            yaw = tag.getSkew();
//            if (!tags.isEmpty() && !tag.getTargetCorners().isEmpty() && !tag.getTargetCorners().get(0).isEmpty()) rightDelta = tag.getTargetCorners().get(0).get(0) - tag.getTargetCorners().get(1).get(0);

//            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    LLResultTypes.FiducialResult tag = tags.get(0);

                    // 3D camera space pose
                    Pose3D campose = tag.getTargetPoseCameraSpace();
                    if (campose != null) {
                        double x   = campose.getPosition().x;
                        double y   = campose.getPosition().y;
                        double z   = campose.getPosition().z;
                        double yaw = campose.getOrientation().getYaw();
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("z", z);
                        telemetry.addData("yaw", yaw);
                    } else {
                        telemetry.addLine("campose is null");
                    }
                } else {
                    telemetry.addLine("no tags detected");
                }
            }

            telemetry.addData("getTa", ta);
            telemetry.addData("getTy", ty);
            telemetry.addData("space:", space);
            telemetry.addData("orientation", orientation);
            telemetry.addData("yaw", yaw);
            telemetry.addData("pipeline:", family);
            telemetry.update();
        }
    }
}