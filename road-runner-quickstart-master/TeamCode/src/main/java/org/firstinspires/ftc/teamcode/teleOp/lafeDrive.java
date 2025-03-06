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
@TeleOp(name = "Drive", group = "TeleOp")
public class lafeDrive extends OpMode {

    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private GamepadEx gamepad1Ex, gamepad2Ex;
    double speed = 1;
    public boolean extension = false;
    public boolean arm = false;
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


        //INITIAL START POSITIONS//
        driveSubsystem.drive(0,0,0, 0);
        //collectionSubsystem.tilt(0.5);
        collectionSubsystem.CRRetract();
    }

    @Override
    public void loop() {
        double looptimeStart = elapsedTime.milliseconds();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        // === Speed Controls ===
        if(gamepad1Ex.getButton(GamepadKeys.Button.Y)){
            speed = .5;
        }
        if(gamepad1Ex.getButton(GamepadKeys.Button.X)){
            speed = 1;
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
            collectionSubsystem.tiltCollect();
            extension = false;
        }



        else if(gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) && arm == false){
            collectionSubsystem.lafeExtend();
            collectionSubsystem.tiltCollect();
            extension = true;
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
        if (gamepad2Ex.getButton(GamepadKeys.Button.Y) && extension == false) {
            depositSubsystem.closeClaw(); // Close claw
            depositSubsystem.tiltPlacespec();
            depositSubsystem.armPlace();
            arm = true;
            collect = 2;
        }



        if (gamepad2Ex.getButton(GamepadKeys.Button.A)&& extension == false) {
            depositSubsystem.openClaw();
            depositSubsystem.tiltPlace();// Open claw
            depositSubsystem.armCollectSpec();
            arm = true;
            collect  = 1;
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();
            depositSubsystem.openClaw();
            arm = false;
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
            if(extension == false) {
                depositSubsystem.armPlace();
                arm = true;
            }

        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 1000){
            if(extension == false) {
                depositSubsystem.tiltPlaceSpec();
                arm = true;
            }
        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 250 && depositSubsystem.liftMotor1.getCurrentPosition()<450){
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();
            arm = false;
        }



        // Update subsystems
        depositSubsystem.updateSlide();

        depositSubsystem.updateTelemetry();
        dashboardTelemetry.addData("Loop Time(MS): ", elapsedTime.milliseconds()-looptimeStart );
        dashboardTelemetry.update();
        telemetry.addData("Extension: " , extension);
        collectionSubsystem.updateTelemetry();


    }
}
