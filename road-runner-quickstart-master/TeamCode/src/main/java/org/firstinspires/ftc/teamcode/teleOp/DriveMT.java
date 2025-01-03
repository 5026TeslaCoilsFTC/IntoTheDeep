package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    double speed = 1;
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
        collectionSubsystem.retractFull();
    }

    @Override
    public void loop() {

        // === Speed Controls ===
        if(gamepad1Ex.getButton(GamepadKeys.Button.Y)){
            speed = 1;
        }
        if(gamepad1Ex.getButton(GamepadKeys.Button.X)){
            speed = 0.5;
        }

        // === Drive Controls ===
        double drive = -gamepad1Ex.getLeftY(); // Forward/Backward
        double strafe = gamepad1Ex.getLeftX(); // Left/Right
        double turn = gamepad1Ex.getRightX();  // Rotation
        driveSubsystem.drive(drive, strafe, turn, speed);

        // === Collection Controls ===
        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            collectionSubsystem.collect();
        } else if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            collectionSubsystem.reverseCollection();
        } else if (gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            collectionSubsystem.retract();
        } else {
            collectionSubsystem.stopCollection();
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            collectionSubsystem.extend();
        }
        else if(gamepad1Ex.getButton(GamepadKeys.Button.A)){
            collectionSubsystem.retractFull();
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            collectionSubsystem.tiltCollect();
        }

        else if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            collectionSubsystem.tiltRetract();
        }

        // === Deposit Controls ===
        // Handle slide positions with D-Pad
        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
            depositSubsystem.setTargetSlide(1000); // Top position (adjust value based on your setup)
        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            depositSubsystem.setTargetSlide(0); // Zero position
        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            depositSubsystem.setTargetSlide(depositSubsystem.getTargetSlide() + 50); // Middle position (adjust value based on your setup)
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            depositSubsystem.setTargetSlide(depositSubsystem.getTargetSlide() - 50);
        }

        // Handle tilt and claw with A/B buttons
        if (gamepad2Ex.getButton(GamepadKeys.Button.Y)) {
            depositSubsystem.closeClaw(); // Close claw
            depositSubsystem.setTiltSPES();
            collect = 2;
        }



        if (gamepad2Ex.getButton(GamepadKeys.Button.A)) {
            depositSubsystem.openClaw(); // Open claw
            depositSubsystem.setTiltSpecCollect();
            collect  = 1;
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            depositSubsystem.setTiltCollect();
            depositSubsystem.openClaw();
            collect = 0;

        }

        if(collectionSubsystem.getExtensionPoz()>collectionSubsystem.MIN_EXTENSION){
            collectionSubsystem.tiltCollect();
        }

        

        // Update subsystems
        depositSubsystem.updateSlide();
        depositSubsystem.updateTelemetry();
        collectionSubsystem.updateTelemetry();
        depositSubsystem.updateWrist(collect);
    }
}
