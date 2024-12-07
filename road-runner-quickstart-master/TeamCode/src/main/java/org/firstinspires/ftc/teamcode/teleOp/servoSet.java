package org.firstinspires.ftc.teamcode.teleOp;
//+
// .
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@TeleOp
public class servoSet extends LinearOpMode {
    //    public Servo    L1  = null;
//    public Servo   L2  = null;
    public Servo tiltServo2 = null;
    public Servo tiltServo1 = null;
    public Servo intake = null;
    public static double servoPosition = 0.5;
    public static double servo2Position =0.5;
    public Servo heightS;

    @Override
    public void runOpMode() throws InterruptedException {

//        L1  = hardwareMap.get(Servo.class, "LIntake");
//        L2  = hardwareMap.get(Servo.class, "RIntake");
        tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
        tiltServo2 = hardwareMap.get(Servo.class, "tiltServo2");
        tiltServo1.setDirection(Servo.Direction.REVERSE);
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        wristR = hardwareMap.get(Servo.class, "wristR");
        servoPosition = 0.5;
        servo2Position = 0.5;

        waitForStart();


        while (!isStopRequested()) {

            if (gamepad1.dpad_up) {
                servoPosition += 0.0003;
                servo2Position += 0.0003;
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.0003;
                servo2Position -= 0.0003;
            }

//            if (gamepad1.dpad_left) {
//                servo2Position += 0.0003;
//            } else if (gamepad1.dpad_right) {
//                servo2Position -= 0.0003;
//            }
//            if (gamepad1.right_bumper) {
//                servoPosition += 0.0003;
//            } else if (gamepad1.left_bumper) {
//                servoPosition -= 0.0003;
            }
              tiltServo1.setPosition(servoPosition);
              tiltServo2.setPosition(servo2Position);
           // heightS.setPosition(servoPosition);
            //intake.setPosition(servo2Position);
//            wristR.setPosition(servo2Position);
            telemetry.addData("position", servoPosition);
            telemetry.addData("position2", servo2Position);

            telemetry.update();
        }

    }
