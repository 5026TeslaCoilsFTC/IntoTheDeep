package org.firstinspires.ftc.teamcode.subsytem;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlinkinSubsystem {

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private Telemetry telemetry;

    public BlinkinSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK); // Start with LEDs off
    }

    /**
     * Sets the LED pattern.
     * @param pattern The BlinkinPattern corresponding to the desired LED effect.
     */
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
        blinkinLedDriver.setPattern(pattern);
        telemetry.addData("Blinkin LED Pattern", pattern.toString());
        telemetry.update();
    }

    // Predefined LED states for easy access
    public void turnOff() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setGreen() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setPurple() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    public void setAlternatePurpleGreen() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
    }
}
