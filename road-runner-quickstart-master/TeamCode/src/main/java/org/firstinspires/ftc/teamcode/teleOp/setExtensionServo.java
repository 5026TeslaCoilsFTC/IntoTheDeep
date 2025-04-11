package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Set Extension Servos to 0", group = "TeleOp")
public class setExtensionServo extends OpMode {

    private Servo  extensionServo1, extensionServo2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Initialize the extension servos
        try {
            extensionServo1 = hardwareMap.get(Servo.class, "extensionServo1");
            //extensionServo2 = hardwareMap.get(CRServo.class, "extensionServo2");
            telemetry.addData("Servo Initialization", "Servos found and initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "One or both extension servos not found in hardware map!");
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        // Set the extension servos to position 0


        telemetry.addData("Action", "Extension servos set to 0");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Extension servos are at 0");

        telemetry.update();
    }
}
