package org.firstinspires.ftc.teamcode.subsytem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CollectionSubsystem {
    private Servo extensionServo1, extensionServo2, tiltServo;
    private DcMotor collectionMotor;
    private Telemetry telemetry;

    private double extensionPosition = 0.0;
    private final double MIN_EXTENSION = 0.4;
    private final double MAX_EXTENSION = 0.6464;
    private final double INCREMENT = 0.05;


    public CollectionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        extensionServo1 = hardwareMap.get(Servo.class, "ex" +
                "tensionServo1");
        extensionServo2 = hardwareMap.get(Servo.class, "extensionServo2");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        collectionMotor = hardwareMap.get(DcMotor.class, "intake");
        collectionMotor.setDirection(DcMotor.Direction.REVERSE);
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
    public void retractFull(){
        extensionServo1.setPosition(.47);
        extensionServo2.setPosition(.48);
    }

    public void tilt(double position) {
        tiltServo.setPosition(position);
    }

    public void collect() {
        collectionMotor.setPower(1.0);
    }

    public void stopCollection() {
        collectionMotor.setPower(0);
    }
    public void reverseCollection(){
        collectionMotor.setPower(-1);
    }

    public void updateTelemetry() {
        telemetry.addData("Extension Position", extensionPosition);
        telemetry.addData("Tilt Servo", tiltServo.getPosition());
        telemetry.addData("Collection Motor Power", collectionMotor.getPower());
    }
    public void extendToPoz(double pos1, double pos2){
        extensionServo1.setPosition(pos1);
        extensionServo2.setPosition(pos2);
    }
    public double getExtensionPoz(){
        return extensionPosition;
    }
}
