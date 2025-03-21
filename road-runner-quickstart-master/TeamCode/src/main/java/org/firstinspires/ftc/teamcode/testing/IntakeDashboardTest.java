package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Config
@TeleOp(name = "Intake Dashboard Test", group = "Testing")
public class IntakeDashboardTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private DcMotor intakeMotor;
    private Servo extensionServo1, extensionServo2, tiltServo1, tiltServo2;
    private Servo clawServo; // Outtake claw servo
    private GamepadEx gamepad1Ex;
    private FtcDashboard dashboard;

    private boolean objectDetected = false;
    private boolean intakeRunning = false;
    private boolean intakeRetracted = false;

    // FTC Dashboard adjustable values
    public static double extensionPosition = 0.5; // Extend servo position
    public static double tiltPosition = 0.5; // Tilt servo position
    public static double clawClosePosition = 0.0; // Claw closed position
    public static double clawOpenPosition = 0.3; // Claw open position
    public static double intakePower = 1.0; // Intake motor speed
    public static int redThreshold = 10;
    public static int blueThreshold = 10;
    public static int yellowThreshold = 10;

    @Override
    public void runOpMode() {
        // Initialize hardware
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        extensionServo1 = hardwareMap.get(Servo.class, "extensionServo1");
        extensionServo2 = hardwareMap.get(Servo.class, "extensionServo2");
        tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
        tiltServo1.setDirection(Servo.Direction.REVERSE);
        clawServo = hardwareMap.get(Servo.class, "clawServo"); // Outtake claw

        // Motor setup
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad1Ex = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Intake Dashboard Test Initialized");
        telemetry.update();

        waitForStart();
        colorSensor.enableLed(false);
        while (opModeIsActive()) {
            // Read color sensor values
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green(); // Approximate yellow detection

            // Determine if object is detected using FTC Dashboard thresholds
            boolean newObjectDetected = (red > redThreshold || blue > blueThreshold || green > yellowThreshold);

            // If an object is detected, start the intake and keep running
            if (newObjectDetected) {
                objectDetected = true;
                intakeRunning = true;
            }

            // If object was detected and is now gone, stop the intake
            if (objectDetected && !newObjectDetected) {
                intakeMotor.setPower(0);
                intakeRunning = false;
                objectDetected = false;
            }

            // Run intake while object is detected
            if (intakeRunning) {
                intakeMotor.setPower(intakePower);
            }

            // Manual override for intake motor control
            if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                intakeMotor.setPower(intakePower); // Forward
            } else if (gamepad1Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                intakeMotor.setPower(-intakePower); // Reverse
            }

            // Adjust servos via FTC Dashboard
           // extensionServo1.setPosition(extensionPosition);
            extensionServo2.setPosition(extensionPosition);
            tiltServo1.setPosition(tiltPosition);


            // Auto-flip intake down when extended past threshold
//            if (extensionPosition >= 0.4) {
//                tiltServo1.setPosition(0.83);
//
//                telemetry.addData("Intake", "Flipped Down");
//            }

            // Auto-retract when block is detected
            if (objectDetected) {
//                extensionServo1.setPosition(0.765);
//                extensionServo2.setPosition(0.765);
                intakeRetracted = true;
                telemetry.addData("Intake", "Retracted");
            }

            // Close claw when intake retracts


            // Telemetry output for debugging
            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);
            telemetry.addData("Green (Yellow)", green);
            telemetry.addData("Object Detected", objectDetected);
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Intake Retracted", intakeRetracted);
            telemetry.addData("Extension Position", extensionPosition);
            telemetry.addData("Tilt Position", tiltPosition);
            telemetry.update();
        }
    }
}
