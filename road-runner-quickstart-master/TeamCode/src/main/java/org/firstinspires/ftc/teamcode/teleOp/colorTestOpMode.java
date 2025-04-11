package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
@Config
@TeleOp(name = "Color Classification Test", group = "Testing")
public class colorTestOpMode extends LinearOpMode {
    private ColorSensor colorSensor;
    private FtcDashboard dashboard;

    public static double[] yellowColor = {900, 1300, 400};
    public static double[] blueColor = {300, 400, 700};
    public static double[] redColor = {900, 600, 300};

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready to classify!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            String color = classifyColor(red, green, blue);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Red", red);
            packet.put("Green", green);
            packet.put("Blue", blue);
            packet.put("Classified Color", color);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Raw R/G/B", "%d / %d / %d", red, green, blue);
            telemetry.addData("Classified", color);
            telemetry.update();

            sleep(50);
        }
    }

    public double colorDistance(double[] a, double[] b) {
        return Math.sqrt(
                Math.pow(a[0] - b[0], 2) +
                        Math.pow(a[1] - b[1], 2) +
                        Math.pow(a[2] - b[2], 2)
        );
    }

    public String classifyColor(int red, int green, int blue) {
        double[] current = {red, green, blue};

        double yellowDist = colorDistance(current, yellowColor);
        double blueDist = colorDistance(current, blueColor);
        double redDist = colorDistance(current, redColor);

        if (yellowDist < blueDist && yellowDist < redDist) {
            return "yellow";
        } else if (blueDist < redDist) {
            return "blue";
        } else {
            return "red";
        }
    }
}
