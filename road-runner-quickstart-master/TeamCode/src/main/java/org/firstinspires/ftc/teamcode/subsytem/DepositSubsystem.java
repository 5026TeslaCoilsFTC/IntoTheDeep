package org.firstinspires.ftc.teamcode.subsytem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDController;
@Config
public class DepositSubsystem {
    public Servo tiltServo1, tiltServo2, clawTiltServo, clawServo;
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

    // Servo positions for tilt and claw
    private double tiltPosition = 0.0;
    private final double MIN_TILT = 0.0;
    private final double MAX_TILT = 1.0;
    private final double tiltClawCollectSpec = .0;
    private final double tiltClawCollect = 0.8;
    private final double tiltClawPlaceSpec = .55;
    private final double tiltClawPlace = 0.71;
    public static double tiltSpec = .087;
    public static double tiltPlace = .72;
    public final double tiltCollectSpecPos = 1;
    public final double tiltCollectPos = .06;
    public DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        tiltServo1 = hardwareMap.get(Servo.class, "tiltOutServo1");
        tiltServo2 = hardwareMap.get(Servo.class, "tiltOutServo2");
        clawTiltServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo = hardwareMap.get(Servo.class, "clawTiltServo");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        tiltServo1.setDirection(Servo.Direction.REVERSE);
        // Reset and configure lift motors
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
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
        setTilt(tiltPlace);
    }
    public void setTiltSPES(){
        setTilt(tiltSpec);
    }
    public void setTiltCollect(){
        setTilt(tiltCollectPos);
    }

    public void setTiltCollect2(){
        setTilt(.3860);
    }
    public void setTiltCollect3(){
        setTilt(.2);
    }
    public void setTiltSpecCollect(){
        setTilt(tiltCollectSpecPos);
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
    public void tiltPlacec(){clawTiltServo.setPosition(tiltClawPlace+.18);}
    public void tiltPlacespec(){clawTiltServo.setPosition(tiltClawPlace+.1);}
    public double getLiftPoz(){double averageliftPos =liftMotor1.getCurrentPosition()+liftMotor2.getCurrentPosition()/2; return averageliftPos;}
    public void updateTelemetry() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry.addData("Tilt Servo 1", tiltServo1.getPosition());
        telemetry.addData("Tilt Servo 2", tiltServo2.getPosition());
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
