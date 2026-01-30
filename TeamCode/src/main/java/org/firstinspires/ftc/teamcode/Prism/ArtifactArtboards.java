package org.firstinspires.ftc.teamcode.Prism;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArtifactArtboards extends LinearOpMode{

    GoBildaPrismDriver prism;
    PrismAnimations.Solid right1 = new PrismAnimations.Solid(Color.WHITE);
    PrismAnimations.Solid back1 = new PrismAnimations.Solid(Color.WHITE);
    PrismAnimations.Solid left = new PrismAnimations.Solid(Color.WHITE);
    PrismAnimations.Solid back2 = new PrismAnimations.Solid(Color.WHITE);
    PrismAnimations.Solid right2 = new PrismAnimations.Solid(Color.WHITE);

    @Override
    public  void runOpMode() throws InterruptedException {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        right1.setBrightness(100);
        right1.setIndexes(0, 3);
        back1.setBrightness(100);
        back1.setIndexes(4, 7);
        left.setBrightness(100);
        left.setIndexes(8, 15);
        back2.setBrightness(100);
        back2.setIndexes(16, 19);
        right2.setBrightness(100);
        right2.setIndexes(20, 23);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) { //0 PPP Left -> Right
                right1.setPrimaryColor(Color.PURPLE);
                back1.setPrimaryColor(Color.PURPLE);
                left.setPrimaryColor(Color.PURPLE);
                back2.setPrimaryColor(Color.PURPLE);
                right2.setPrimaryColor(Color.PURPLE);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_0);
            }
            if (gamepad1.b) { //1 PPG
                right1.setPrimaryColor(Color.GREEN);
                back1.setPrimaryColor(Color.PURPLE);
                left.setPrimaryColor(Color.PURPLE);
                back2.setPrimaryColor(Color.PURPLE);
                right2.setPrimaryColor(Color.GREEN);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_1);
            }
            if (gamepad1.x) { //2 PGG
                right1.setPrimaryColor(Color.GREEN);
                back1.setPrimaryColor(Color.GREEN);
                left.setPrimaryColor(Color.PURPLE);
                back2.setPrimaryColor(Color.PURPLE);
                right2.setPrimaryColor(Color.GREEN);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_2);
            }
            if (gamepad1.y) {//3 PGP
                right1.setPrimaryColor(Color.PURPLE);
                back1.setPrimaryColor(Color.GREEN);
                left.setPrimaryColor(Color.PURPLE);
                back2.setPrimaryColor(Color.GREEN);
                right2.setPrimaryColor(Color.PURPLE);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_3);
            }
            if (gamepad1.dpad_down) {//4 GPP
                right1.setPrimaryColor(Color.PURPLE);
                back1.setPrimaryColor(Color.PURPLE);
                left.setPrimaryColor(Color.GREEN);
                back2.setPrimaryColor(Color.PURPLE);
                right2.setPrimaryColor(Color.PURPLE);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_4);
            }
            if (gamepad1.dpad_left) {//5 GGP
                right1.setPrimaryColor(Color.PURPLE);
                back1.setPrimaryColor(Color.GREEN);
                left.setPrimaryColor(Color.GREEN);
                back2.setPrimaryColor(Color.GREEN);
                right2.setPrimaryColor(Color.PURPLE);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_5);
            }
            if (gamepad1.dpad_right) {//6 GPG
                right1.setPrimaryColor(Color.GREEN);
                back1.setPrimaryColor(Color.PURPLE);
                left.setPrimaryColor(Color.GREEN);
                back2.setPrimaryColor(Color.PURPLE);
                right2.setPrimaryColor(Color.GREEN);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_6);
            }
            if (gamepad1.dpad_up) {//7 GGG
                right1.setPrimaryColor(Color.GREEN);
                back1.setPrimaryColor(Color.GREEN);
                left.setPrimaryColor(Color.GREEN);
                back2.setPrimaryColor(Color.GREEN);
                right2.setPrimaryColor(Color.GREEN);
                prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_7);
            }
        }
    }
}
