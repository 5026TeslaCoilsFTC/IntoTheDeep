package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Testing")
public class ColorSensorTest extends OpMode {
    private ColorSensor intakeColorSensor;
    private ColorSensor transferColorSensor;

    @Override
    public void init() {
        // Initialize color sensors
        intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColor");
        transferColorSensor = hardwareMap.get(ColorSensor.class, "transferColor");

        telemetry.addLine("Color Sensor Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read color values from intake sensor
        int intakeRed = intakeColorSensor.red();
        int intakeBlue = intakeColorSensor.blue();
        int intakeGreen = intakeColorSensor.green();
        int intakeBrightness = intakeRed + intakeBlue + intakeGreen;

        // Read color values from transfer sensor
        int transferRed = transferColorSensor.red();
        int transferBlue = transferColorSensor.blue();
        int transferGreen = transferColorSensor.green();
        int transferBrightness = transferRed + transferBlue + transferGreen;

        // Display values on the Driver Station
        telemetry.addLine("== Intake Sensor ==");
        telemetry.addData("Red", intakeRed);
        telemetry.addData("Blue", intakeBlue);
        telemetry.addData("Green", intakeGreen);
        telemetry.addData("Brightness", intakeBrightness);

        telemetry.addLine("== Transfer Sensor ==");
        telemetry.addData("Red", transferRed);
        telemetry.addData("Blue", transferBlue);
        telemetry.addData("Green", transferGreen);
        telemetry.addData("Brightness", transferBrightness);

        telemetry.update();
    }
}
