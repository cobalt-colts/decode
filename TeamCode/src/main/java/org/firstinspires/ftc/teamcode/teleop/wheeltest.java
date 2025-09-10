package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@Config
@TeleOp
public class wheeltest extends LinearOpMode {
    public static DcMotor currentwheel;

    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                backLeftMotor.setPower(1);
            } else {
                backLeftMotor.setPower(0);
            }
            if (gamepad1.b) {
                frontLeftMotor.setPower(1);
            } else {
                frontLeftMotor.setPower(0);
            }
            if (gamepad1.x) {
                frontRightMotor.setPower(1);
            } else {
                frontRightMotor.setPower(0);
            }
            if (gamepad1.y) {
                backRightMotor.setPower(1);
            } else {
                backRightMotor.setPower(0);
            }
        }
    }

}
