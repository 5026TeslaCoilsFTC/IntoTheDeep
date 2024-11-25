package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;

@TeleOp(name = "FTCLib TeleOp", group = "TeleOp")
public class Drive extends OpMode {

    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private GamepadEx gamepad1Ex, gamepad2Ex;

    @Override
    public void init() {
        // Initialize subsystems and gamepads
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        // === Drive Controls ===
        double drive = -gamepad1Ex.getLeftY(); // Forward/Backward
        double strafe = gamepad1Ex.getLeftX(); // Left/Right
        double turn = gamepad1Ex.getRightX();  // Rotation

        driveSubsystem.drive(drive, strafe, turn, 1);

        // === Collection Controls ===
        if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            collectionSubsystem.collect();
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

        // === Deposit Controls ===
        // Handle slide positions with D-Pad
//        if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)) {
//            depositSubsystem.setTargetSlide(1000); // Top position (adjust value based on your setup)
//        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            depositSubsystem.setTargetSlide(0); // Zero position
//        } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_LEFT)) {
//            depositSubsystem.setTargetSlide(500); // Middle position (adjust value based on your setup)
//        }
        if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)){
            depositSubsystem.liftsUp(-1);
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)){
            depositSubsystem.liftsDown(-1);
        }
        else{
            depositSubsystem.liftsStop();
        }

        // Handle tilt and claw with A/B buttons
        if (gamepad2Ex.getButton(GamepadKeys.Button.A)) {
            depositSubsystem.setTiltPlace();
            depositSubsystem.closeClaw(); // Close claw
        } else if (gamepad2Ex.getButton(GamepadKeys.Button.B)) {
            depositSubsystem.setTiltCollect();
            depositSubsystem.openClaw(); // Open claw
            depositSubsystem.tiltCollect();
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.X)){
            depositSubsystem.tiltPlace();
        }
        // Update subsystems
        //depositSubsystem.updateSlide();
        depositSubsystem.updateTelemetry();
        collectionSubsystem.updateTelemetry();
    }
}
