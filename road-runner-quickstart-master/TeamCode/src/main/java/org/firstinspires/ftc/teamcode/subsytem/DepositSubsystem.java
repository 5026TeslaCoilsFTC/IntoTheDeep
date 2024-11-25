package org.firstinspires.ftc.teamcode.subsytem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDController;

public class DepositSubsystem {
    private Servo tiltServo1, tiltServo2, clawTiltServo, clawServo;
    private DcMotor liftMotor1, liftMotor2;
    private Telemetry telemetry;

    private PIDController slideController;

    // PID coefficients for slide
    private double pSlide = 0.01, iSlide = 0.0, dSlide = 0.0;

    // Feedforward coefficient
    private double f = 0.1;

    // Slide target position
    private double targetSlide = 0.0;

    // Servo positions for tilt and claw
    private double tiltPosition = 0.0;
    private final double MIN_TILT = 0.0;
    private final double MAX_TILT = 1.0;
    private final double tiltClawCollect = .43;
    private final double tiltClawPlace = .35;

    public DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
        tiltServo2 = hardwareMap.get(Servo.class, "tiltServo2");
        clawTiltServo = hardwareMap.get(Servo.class, "clawTiltServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        // Reset and configure lift motors
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID controller
        slideController = new PIDController(pSlide, iSlide, dSlide);
    }

    public void setTargetSlide(double target) {
        this.targetSlide = target;
    }

    public double getTargetSlide() {
        return this.targetSlide;
    }

    public void setTilt(double position) {
        tiltPosition = Math.max(MIN_TILT, Math.min(MAX_TILT, position));
        tiltServo1.setPosition(tiltPosition);
        tiltServo2.setPosition(tiltPosition);
    }
    public void setTiltPlace(){
        setTilt(.6416);
    }
    public void setTiltSPES(){
        setTilt(.8);
    }

    public void setTiltCollect2(){
        setTilt(.3860);
    }
    public void setTiltCollect3(){
        setTilt(.2);
    }
    public void setTiltCollect(){
        setTilt(.09);
    }

    public double getTiltPosition() {
        return this.tiltPosition;
    }

    public void updateSlide() {
        int currentSlidePos = (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition()) / 2;

        double pid = slideController.calculate(currentSlidePos, targetSlide);
        double ff = f;

        double power = pid + ff;
        power = Math.max(-1.0, Math.min(1.0, power));

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        telemetry.addData("Slide Target", targetSlide);
        telemetry.addData("Slide Position", currentSlidePos);
        telemetry.addData("Slide Power", power);
    }

    public void openClaw() {
        clawServo.setPosition(.4);
    }

    public void closeClaw() {
        clawServo.setPosition(.2);
    }
    public void tiltCollect(){
        clawTiltServo.setPosition(tiltClawCollect);
    }
    public void tiltPlace(){
        clawTiltServo.setPosition(tiltClawPlace);
    }
    public void liftsUp(double speed){
        liftMotor1.setPower(speed);
        liftMotor2.setPower(-speed);
    }
    public void liftsDown(double speed){
        liftMotor1.setPower(-speed);
        liftMotor2.setPower(speed);
    }
    public void liftsStop(){
        liftMotor2.setPower(0);
        liftMotor1.setPower(0);
    }
    public void updateTelemetry() {
        telemetry.addData("Tilt Servo 1", tiltServo1.getPosition());
        telemetry.addData("Tilt Servo 2", tiltServo2.getPosition());
        telemetry.addData("Claw Tilt Servo", clawTiltServo.getPosition());
        telemetry.addData("Claw Servo", clawServo.getPosition());
    }
}
