package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Config
public class colorClassifierTest {
    private final ColorSensor colorSensor;

    // Tunable normalized thresholds
    public static double yellowRedMin = 0.405;
    public static double yellowGreenMin = 0.45;

    public static double blueBlueMin = .595;
    public static double blueRedMax = 0.35;
    public static double blueGreenMax = 0.35;

    public static double redRedMin = 0.545;
    public static double redGreenMax = .3;
    public static double alphaMin = 700;

    public enum DetectedColor {
        YELLOW, BLUE, RED, NONE
    }

    public colorClassifierTest(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }
    public boolean detectValidObject () {
        return getColor() != DetectedColor.NONE;
    }
    public DetectedColor getColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        double total = red + green + blue;
        if (total == 0) return DetectedColor.NONE; // avoid divide by zero
        double normRed = red / total;
        double normGreen = green / total;
        double normBlue = blue / total;


             if (normBlue >= blueBlueMin && normRed <= blueRedMax && normGreen <= blueGreenMax) {
                return DetectedColor.BLUE;
            } else if (normRed >= redRedMin && normGreen <= redGreenMax) {
                return DetectedColor.RED;
            } else if (colorSensor.alpha()> alphaMin) {
                 return DetectedColor.YELLOW;
             } else {
                return DetectedColor.NONE;
            }





    }}