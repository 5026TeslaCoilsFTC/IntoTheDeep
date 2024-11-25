package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Autonomous(name = "All Park", group = "TeleOp")
public class MTParkAll extends LinearOpMode {

    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;

    private ElapsedTime runtime = new ElapsedTime();

    private GamepadEx gamepad1Ex;

    private int startPos = -1; //LEFT (1) or RIGHT (2)

    private int delay = -1; //SECONDS

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize subsystems and gamepads
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);

        gamepad1Ex = new GamepadEx(gamepad1);

        while(startPos == -1){
            if(gamepad1Ex.getButton(GamepadKeys.Button.X)){
                startPos = 1;
                telemetry.addData("Starting Position: ", "LEFT");
            }
            if(gamepad1Ex.getButton(GamepadKeys.Button.B)){
                startPos = 2;
                telemetry.addData("Starting Position: ", "RIGHT");
            }
        }

        // Longest possible delay is 22 seconds, longest safe delay is 20 seconds
        while(delay == -1){
            if(gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN)){
                delay = 0;
                telemetry.addData("Delay Time: ", "NO DELAY");
            }
            if(gamepad1Ex.getButton(GamepadKeys.Button.DPAD_RIGHT)){
                delay = 5;
                telemetry.addData("Delay Time: ", "5 SECONDS");
            }
            if(gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP)){
                delay = 10;
                telemetry.addData("Delay Time: ", "10 SECONDS");
            }
            if(gamepad1Ex.getButton(GamepadKeys.Button.DPAD_LEFT)){
                delay = 15;
                telemetry.addData("Delay Time: ", "15 SECONDS");
            }
        }

        telemetry.update();
        collectionSubsystem.tilt(0.5);
        collectionSubsystem.retractFull();



        waitForStart();

        driveSubsystem.driveStop();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < delay)) {
            telemetry.addData("Path", "Pause 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        driveSubsystem.driveStop();

        if(startPos == 1){ //Starting in Left position
            driveSubsystem.driveLeft(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.8)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            driveSubsystem.driveStop();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Pause 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            driveSubsystem.driveStop();

            driveSubsystem.driveRight(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 7
            )) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            driveSubsystem.driveStop();
        }else if(startPos == 2){ // Starting in Right position
            driveSubsystem.driveLeft(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            driveSubsystem.driveStop();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Pause 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            driveSubsystem.driveStop();

            driveSubsystem.driveRight(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 6)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            driveSubsystem.driveStop();
        }



    }
}
