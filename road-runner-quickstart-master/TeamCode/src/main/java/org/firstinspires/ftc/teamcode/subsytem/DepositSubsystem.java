package org.firstinspires.ftc.teamcode.subsytem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDController;
@Config
public class DepositSubsystem {
    public Servo armL, armR, clawTiltServo, clawServo;
    public DcMotor liftMotor1, liftMotor2;
    private Telemetry telemetry;
    private enum Lift {
        liftUp,
        liftDown
    }
    private enum ArmPoz{
        collect,
        collectSpec,
        place
    }
    private PIDController slideController;


    Lift lift = Lift.liftUp;
    ArmPoz armPoz = ArmPoz.collect;

    // PID coefficients for slide
    private double pSlide = 0.003, iSlide = 0.0, dSlide = 0.0;


    // Feedforward coefficient
    private double f = 0.0002;

    // Slide target position
    public static double targetSlide = 0.0;

    public static double targetTilt = 0.0;

    // Servo positions for tilt and claw
    private double tiltPosition = 0.0;
    private final double MIN_TILT = 0.0;
    private final double MAX_TILT = 1.0;
    private final double tiltClawCollectSpec = .0;
    private final double tiltClawCollect = 0.79;
    private final double tiltClawPlaceSpec = .55;
    private final double tiltClawPlace = 0.71;
    public static double tiltSpec = .087;
    public static double tiltPlace = .65;
    public final double tiltCollectSpecPos = .85;
    public final double tiltCollectPos = .035;

    public DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        armL = hardwareMap.get(Servo.class,"armL");
        armR = hardwareMap.get(Servo.class, "armR");
        armL.setDirection(Servo.Direction.REVERSE);
        clawTiltServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo = hardwareMap.get(Servo.class, "clawTiltServo");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        // Reset and configure lift motors
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        // Initialize PID controller
        slideController = new PIDController(pSlide, iSlide, dSlide);

    }

    public void armPlace(){
        armL.setPosition(.64);
        armR.setPosition(.64);
    }
    public void armCollect(){
        armL.setPosition(.17);
        armR.setPosition(.17);
    }
    public void armMiddle(){
        armL.setPosition(.3);
        armR.setPosition(.3);
    }
    public void armCollectSpec() {
        armL.setPosition(.83);
        armR.setPosition(.83);
    }
    public void armHigh(){
        armL.setPosition(.5);
        armR.setPosition(.5);
    }

    public void setTargetSlide(double target) {
        this.targetSlide = target;
    }

    public double getTargetSlide() {
        return this.targetSlide;
    }



    public double getArmTarget(){
        return targetTilt;
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
        clawServo.setPosition(.15);
    }
    public void setClaw(double poz){ clawTiltServo.setPosition(poz);}
    public void closeClaw() {
        clawServo.setPosition(0);
    }
    public void tiltCollectSpec(){
        clawTiltServo.setPosition(tiltClawCollectSpec);
    }
    public void tiltCollect(){clawTiltServo.setPosition(tiltClawCollect);}

    public void tiltPlaceSpec(){
        clawTiltServo.setPosition(tiltClawPlaceSpec);
    }
    public void tiltPlace(){ clawTiltServo.setPosition(tiltClawPlace);}
    public void tiltPlacec(){clawTiltServo.setPosition(.9);}
    public void tiltPlacespec(){clawTiltServo.setPosition(tiltClawPlace+.1);}
    public double getLiftPoz(){double averageliftPos =liftMotor1.getCurrentPosition()+liftMotor2.getCurrentPosition()/2; return averageliftPos;}
    public void updateTelemetry() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Claw Tilt Servo", clawTiltServo.getPosition());
        telemetry.addData("Claw Servo", clawServo.getPosition());
        telemetry.addData("Lift Target", targetSlide);
        telemetry.addData("Lift Positions", liftMotor1.getCurrentPosition());
        telemetry.addData("Lift Error", targetSlide-liftMotor2.getCurrentPosition());
        double liftErrorPercent = targetSlide-(liftMotor2.getCurrentPosition());
        liftErrorPercent /= targetSlide;
        dashboardTelemetry.addData("Lift Error Percentage", liftErrorPercent);
        dashboardTelemetry.addData("Lift Target", targetSlide);
        dashboardTelemetry.addData("Lift Positions", liftMotor2.getCurrentPosition());
        dashboardTelemetry.addData("Lift Error", targetSlide-liftMotor2.getCurrentPosition());
        dashboardTelemetry.addData("Lift Error Percentage", liftErrorPercent);
        dashboardTelemetry.update();
    }
    public void resetLifts(){
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateWrist(int collect){
        switch(lift){
            case liftUp:
                switch (armPoz){
                    case collect:
                        tiltCollect();
                        break;
                    case collectSpec:
                        tiltCollectSpec();
                        break;
                    case place:
                        tiltPlace();
                        break;
                }
            case liftDown:
                switch(armPoz){
                    case collect:
                        tiltCollect();
                        break;
                    case collectSpec:
                        tiltCollectSpec();
                        break;
                    case place:
                        tiltPlaceSpec();
                        break;
                }
        }

        if(getLiftPoz() > 50){
            lift = Lift.liftUp;
        }
        else{
            lift = Lift.liftDown;
        }

        if(collect == 0){
            armPoz = ArmPoz.collect;
        }
        else if(collect == 1){
            armPoz = ArmPoz.collectSpec;
        }
        else{
            armPoz = ArmPoz.place;
        }
        telemetry.addData("Collect Number", collect);


    }

}
