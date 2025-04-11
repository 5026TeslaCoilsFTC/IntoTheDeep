package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Set Arm .5", group = "TeleOp")
public class setaArmTo5 extends OpMode {

    private Servo  armL, armR;
    public static double armPos = 0.5;
    public static double tiltPos = .3;
    public static double wristPos = .79;
    public Servo tiltServo1, clawServo;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Initialize the extension servos
        try {
            armL = hardwareMap.get(Servo.class, "armL");
            armR = hardwareMap.get(Servo.class, "armR");
            tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
            tiltServo1.setDirection(Servo.Direction.REVERSE);
            clawServo = hardwareMap.get(Servo.class, "clawServo");
          armL.setDirection(Servo.Direction.REVERSE);
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
        armL.setPosition(armPos);
        armR.setPosition(armPos);
//        tiltServo1.setPosition(tiltPos);
//        clawServo.setPosition(wristPos);
        telemetry.update();
    }
}
