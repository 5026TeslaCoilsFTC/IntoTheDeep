package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.testing.colorClassifierTest;

@Config
@TeleOp(name = "Color Classifier Test Normalization", group = "Testing")
public class colorClassifierTestOpMode extends LinearOpMode {

    private ColorSensor colorSensor;
    private colorClassifierTest classifier;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        classifier = new colorClassifierTest(colorSensor);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Color Classifier Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int rawRed = colorSensor.red();
            int rawGreen = colorSensor.green();
            int rawBlue = colorSensor.blue();

            double total = rawRed + rawGreen + rawBlue;
            double normRed = total > 0 ? rawRed / total : 0;
            double normGreen = total > 0 ? rawGreen / total : 0;
            double normBlue = total > 0 ? rawBlue / total : 0;

            colorClassifierTest.DetectedColor detectedColor = classifier.getColor();

            // --- Driver Station Telemetry ---
            telemetry.addData("Raw RGB", "%d / %d / %d", rawRed, rawGreen, rawBlue);
            telemetry.addData("Norm RGB", "%.2f / %.2f / %.2f", normRed, normGreen, normBlue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            // --- FTC Dashboard Telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Raw Red", rawRed);
            packet.put("Raw Green", rawGreen);
            packet.put("Raw Blue", rawBlue);
            packet.put("Norm Red", normRed);
            packet.put("Norm Green", normGreen);
            packet.put("Norm Blue", normBlue);
            packet.put("Detected Color", detectedColor.toString());
            dashboard.sendTelemetryPacket(packet);

            sleep(50); // Update rate
        }
    }
}
