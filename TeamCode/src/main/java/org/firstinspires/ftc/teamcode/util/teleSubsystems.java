package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ColorSensor;
import static org.firstinspires.ftc.teamcode.util.posConstants.*;
import static org.firstinspires.ftc.teamcode.util.positions.*;

public class teleSubsystems {
    public static char getColor(ColorSensor a, ColorSensor b, int index) {
        if (a.green() >= greenThreshold || b.green() >= greenThreshold) {
            balls[index] = 'P';
            return 'G';
        }
        else if (a.blue() >= blueThreshold || b.blue() >= blueThreshold) {
            balls[index] = 'P';
            return 'P';
        }
        else {
            balls[index] = 'P';
            return 'P';
        }
    }
}
