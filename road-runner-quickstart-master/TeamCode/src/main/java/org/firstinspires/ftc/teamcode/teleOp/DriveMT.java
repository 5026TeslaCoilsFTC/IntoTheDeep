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
import org.firstinspires.ftc.teamcode.subsytem.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;

@Config
@TeleOp(name = "1 - Worlds TeleOp", group = "TeleOp")
public class DriveMT extends OpMode {

    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private GamepadEx gamepad1Ex, gamepad2Ex;
    double speed = 1;
    public boolean extension = false;

    private BlinkinSubsystem blinkinSubsystem;
    public boolean arm = false;
    public   ElapsedTime elapsedTime = new ElapsedTime();
    public int collect = 0;
    private String allianceColor = "RED"; // Change dynamically if needed
    private boolean collecting = false;
    private boolean ejecting = false;
    public static String preMatchColor = "None";
    public static String mode = "Samples";
    @Override
    public void init() {

        blinkinSubsystem = new BlinkinSubsystem(hardwareMap, telemetry);
        // Initialize subsystems and gamepads
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry, depositSubsystem);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);


        //INITIAL START POSITIONS//
        driveSubsystem.drive(0,0,0, 0);
        //collectionSubsystem.tilt(0.5);

        blinkinSubsystem.setAlternatePurpleGreen();

    }

    @Override
    public void loop() {
        if (gamepad1Ex.getButton(GamepadKeys.Button.B)) {
            mode = "Specimens";
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.X)) {
            mode = "Samples";
        }

        telemetry.addData("Pre-Match Color", preMatchColor);
        telemetry.addData("Mode", mode);

        telemetry.addData("Alliance Color: ", allianceColor);
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
            blinkinSubsystem.setGreen();
            depositSubsystem.closeClaw(); // Close claw
            depositSubsystem.tiltPlacespec();
            depositSubsystem.armPlace();
            arm = true;
            collect = 2;
        }



        if (gamepad2Ex.getButton(GamepadKeys.Button.A)&& extension == false) {
            blinkinSubsystem.setPurple();
            depositSubsystem.openClaw();
            depositSubsystem.tiltPlace();// Open claw
            depositSubsystem.armCollectSpec();
            arm = true;
            collect  = 1;
        }

        if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            blinkinSubsystem.setPurple();
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();
            depositSubsystem.openClaw();
            arm = false;
            collect = 0;

        }


        if(gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            depositSubsystem.tiltPlaceSpec();
        }
        else if(gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();
        }

        if(gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> .1){
            blinkinSubsystem.setPurple();
            depositSubsystem.openClaw();
        }
        else if(gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)> .1) {
            blinkinSubsystem.setGreen();
            depositSubsystem.closeClaw();
        }
        if(gamepad2.dpad_up){
            depositSubsystem.armPlace();
        }




        // Update subsystems
        depositSubsystem.updateSlide();

        depositSubsystem.updateTelemetry();
        dashboardTelemetry.addData("Loop Time(MS): ", elapsedTime.milliseconds()-looptimeStart );
        dashboardTelemetry.update();
        telemetry.addData("Extension: " , extension);
        collectionSubsystem.updateState(
                gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)

        );

        telemetry.update();

    }
}
