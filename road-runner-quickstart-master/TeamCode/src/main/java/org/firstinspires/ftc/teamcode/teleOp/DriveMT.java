package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;

@Config
@TeleOp(name = "1 - MT TeleOp", group = "TeleOp")
public class DriveMT extends OpMode {

    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private GamepadEx gamepad1Ex, gamepad2Ex;
    public AnalogInput extensionServo;
   //ublic AnalogInput extensionServo;
    double speed = .5;
    public   ElapsedTime elapsedTime = new ElapsedTime();
    public int collect = 0;
    @Override
    public void init() {

        // Initialize subsystems and gamepads
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        extensionServo = hardwareMap.get(AnalogInput.class, "extensionServo");

        //INITIAL START POSITIONS//
        driveSubsystem.drive(0,0,0, 0);
        //collectionSubsystem.tilt(0.5);

    }

    @Override
    public void loop() {
        double looptimeStart = elapsedTime.milliseconds();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        // === Speed Controls ===
        if(gamepad1Ex.getButton(GamepadKeys.Button.Y)){
            speed = .75;
        }
        if(gamepad1Ex.getButton(GamepadKeys.Button.X)){
            speed = 0.5;
        }

        // === Drive Controls ===
        double drive = gamepad1Ex.getLeftY(); // Forward/Backward
        double strafe = gamepad1Ex.getLeftX(); // Left/Right
        double turn = gamepad1Ex.getRightX();  // Rotation
        driveSubsystem.drive(drive, strafe, turn, speed);

        // === Collection Controls ===
        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            collectionSubsystem.collect();
        } else if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            collectionSubsystem.reverseCollection();
        }  else {
            collectionSubsystem.stopCollection();
        }

      if (gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            collectionSubsystem.CRRetract();
        }

        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            collectionSubsystem.CRExtend();
        }
        else{
            collectionSubsystem.Stop();
      }


        if (gamepad2Ex.getButton(GamepadKeys.Button.B)) {
            collectionSubsystem.tiltCollect();
        }

        else if (gamepad2Ex.getButton(GamepadKeys.Button.X)) {
            collectionSubsystem.tiltRetract();
        }

        // === Deposit Controls ===
        // Handle slide positions with D-Pad
        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            depositSubsystem.setTargetSlide(3100); // Top position (adjust value based on your setup)
        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            depositSubsystem.setTargetSlide(0); // Zero position
        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_LEFT) && depositSubsystem.getTargetSlide() < 3200) {
            depositSubsystem.setTargetSlide(depositSubsystem.getTargetSlide() + 50); // Middle position (adjust value based on your setup)
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            depositSubsystem.setTargetSlide(depositSubsystem.getTargetSlide() - 50);
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.START)){
            depositSubsystem.resetLifts();
        }
        // Handle tilt and claw with A/B buttons
        if (gamepad2Ex.getButton(GamepadKeys.Button.Y)) {
            depositSubsystem.closeClaw(); // Close claw
            depositSubsystem.armPlace();
            depositSubsystem.tiltMotor.setTargetPosition(100);
            collect = 2;
        }



        if (gamepad2Ex.getButton(GamepadKeys.Button.A)) {
            depositSubsystem.openClaw(); // Open claw
            depositSubsystem.armCollectSpec();
            depositSubsystem.tiltMotor.setTargetPosition(300);
            collect  = 1;
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            depositSubsystem.armCollect();

            depositSubsystem.openClaw();
            collect = 0;

        }


        if(gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            depositSubsystem.tiltPlaceSpec();
        }

        if(gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> .1){
            depositSubsystem.openClaw();
        }
        else if(gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)> .1) {
            depositSubsystem.closeClaw();
        }

        if(depositSubsystem.liftMotor1.getCurrentPosition()>500 && depositSubsystem.liftMotor1.getCurrentPosition()<750){
 // Close claw
            depositSubsystem.armPlace();

        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 1000){
            depositSubsystem.tiltPlaceSpec();
        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 250 && depositSubsystem.liftMotor1.getCurrentPosition()<450){
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();

        }
        if(depositSubsystem.tiltMotor.getCurrentPosition()<300 && depositSubsystem.liftMotor1.getCurrentPosition()< 200){
            telemetry.addLine("tilt less than 300");
            if(depositSubsystem.tiltMotor.getCurrentPosition()> 150) {
                telemetry.addLine("tilt less than 300 & greater than 150");
                depositSubsystem.tiltPlacespec();
            }
        }
         if(depositSubsystem.tiltMotor.getCurrentPosition()<150 && depositSubsystem.liftMotor1.getCurrentPosition()< 200){
            telemetry.addLine("tilt less than 200");
            if(depositSubsystem.tiltMotor.getCurrentPosition()> 0) {
                telemetry.addLine("tilt less than 200 & greater than zero");
                depositSubsystem.tiltPlace();            }
        }
         if(depositSubsystem.tiltMotor.getCurrentPosition()<600 && depositSubsystem.liftMotor1.getCurrentPosition()< 200){
            telemetry.addLine("tilt less than 600");
            if(depositSubsystem.tiltMotor.getCurrentPosition()> 450) {
                telemetry.addLine("tilt less than 600 & greater than 450");
                depositSubsystem.tiltPlacec();            }
        }
         if(depositSubsystem.tiltMotor.getCurrentPosition() < 300 && depositSubsystem.liftMotor1.getCurrentPosition()> 3000){
            depositSubsystem.tiltPlaceSpec();

        }


        // Update subsystems
        depositSubsystem.updateSlide();
        depositSubsystem.updateTilt();
        depositSubsystem.updateTelemetry();
        dashboardTelemetry.addData("Loop Time(MS): ", elapsedTime.milliseconds()-looptimeStart );
        dashboardTelemetry.update();
        telemetry.addData("Extension Servo", extensionServo);
        collectionSubsystem.updateTelemetry();


    }
}
