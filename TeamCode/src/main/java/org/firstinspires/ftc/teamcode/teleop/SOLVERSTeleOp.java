package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.util.positions;
import org.firstinspires.ftc.teamcode.util.solvers.subsystems;

@TeleOp(name = "SOLVERS TeleOp")
public class SOLVERSTeleOp extends CommandOpMode {

    private subsystems.Robot robot;
    private GamepadEx driver;
    private boolean teleopPipelineConfigured = false;

    @Override
    public void initialize() {
        subsystems.teleop = true;
        subsystems.start = false;
        subsystems.far = false;
        positions.flyWheelCorrect = 0;

        robot = new subsystems.Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);

        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(robot.index.launch1);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(robot.index.launch2);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(robot.index.launch3);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(robot.index.closeunsortedlaunch);
        new GamepadButton(driver, GamepadKeys.Button.OPTIONS).whenPressed(robot.turret.home);

        robot.drive.setDefaultCommand(robot.drive.driverControl(driver));

        schedule(new RunCommand(() -> {
            if (!subsystems.start) {
                subsystems.start = true;
                subsystems.teleop = true;
            }

            if (!subsystems.start || !subsystems.teleop) {
                robot.intake.intakeMotor.setPower(0);
            } else if (gamepad1.left_bumper) {
                robot.intake.intakeMotor.setPower(0.5);
            } else {
                robot.intake.intakeMotor.setPower(-1);
            }

            if (!teleopPipelineConfigured && subsystems.Thrower.limelight != null) {
                subsystems.Thrower.limelight.pipelineSwitch(positions.redAlliance ? 2 : 3);
                teleopPipelineConfigured = true;
            }

            telemetry.addData("intake", gamepad1.left_bumper ? "OUT" : "IN");
            telemetry.update();
        }));
    }

    @Override
    public void end() {
        if (robot != null) {
            robot.intake.intakeMotor.setPower(0);
            robot.drive.stop();
        }
        subsystems.start = false;
        subsystems.teleop = false;
        teleopPipelineConfigured = false;
    }
}
