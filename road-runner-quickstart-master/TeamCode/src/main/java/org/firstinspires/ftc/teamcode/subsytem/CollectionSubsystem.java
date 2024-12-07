package org.firstinspires.ftc.teamcode.subsytem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CollectionSubsystem {
    private Servo extensionServo1, extensionServo2;
   // private CRServo collectionServo1, collectionServo2;//this sucks in samples
    private Telemetry telemetry;

    private double extensionPosition = 0.0;
    private final double MIN_EXTENSION = 0.4;
    private final double MAX_EXTENSION = 0.6464;
    private final double INCREMENT = 0.005;

    private final double tiltCollectPoz = 0;

    private final double tiltRetractPoz = 1;

    public CollectionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        extensionServo1 = hardwareMap.get(Servo.class, "ex" +
                "tensionServo1");
        extensionServo2 = hardwareMap.get(Servo.class, "extensionServo2");
//        tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
//        tiltServo2 = hardwareMap.get(Servo.class, "tiltServo2");
//        collectionServo1 = hardwareMap.get(CRServo.class, "intake");
//        collectionServo1.setDirection(CRServo.Direction.REVERSE);
//        collectionServo2 = hardwareMap.get(CRServo.class, "intake2");
//        collectionServo2.setDirection(CRServo.Direction.REVERSE);
        extensionServo1.setPosition(MIN_EXTENSION);
        extensionServo2.setPosition(MIN_EXTENSION);
    }

    public void extend() {
        extensionPosition = Math.min(extensionPosition + INCREMENT, MAX_EXTENSION);
        extensionServo1.setPosition(extensionPosition);
        extensionServo2.setPosition(extensionPosition);
    }

    public void retract() {
        extensionPosition = Math.max(extensionPosition - INCREMENT, MIN_EXTENSION);
        extensionServo1.setPosition(extensionPosition);
        extensionServo2.setPosition(extensionPosition);
    }

    public void retractFull() {
        extensionServo1.setPosition(.47);
        extensionServo2.setPosition(.48);
    }

//    public void tilt(double position) {
//        tiltServo1.setPosition(position);
//        tiltServo2.setPosition(position);
//    }
//
//    public void tiltCollect(){
//        tiltServo1.setPosition(tiltCollectPoz);
//        tiltServo2.setPosition(tiltCollectPoz);
//    }
//
//    public void tiltRetract(){
//        tiltServo1.setPosition(tiltRetractPoz);
//        tiltServo2.setPosition(tiltRetractPoz);
//    }
//
//
//
//    public void collect() {
//        collectionServo1.setPower(1);
//        collectionServo2.setPower(1);
//    }
//
//    public void stopCollection() {
//        collectionServo1.setPower(1);
//        collectionServo2.setPower(1);
//    }
//
//    public void reverseCollection() {
//        collectionServo1.setPower(-1);
//        collectionServo2.setPower(-1);
//    }

    public void updateTelemetry() {
        telemetry.addData("Extension Position", extensionPosition);
//        telemetry.addData("Tilt Servo", tiltServo1.getPosition());
//        telemetry.addData("Tilt Servo", tiltServo2.getPosition());
    }

    public void extendToPoz(double pos1, double pos2) {
        extensionServo1.setPosition(pos1);
        extensionServo2.setPosition(pos2);
    }

    public double getExtensionPoz() {
        return extensionPosition;
    }

    public double extendToPoz() {
        return extensionPosition;
    }
}

