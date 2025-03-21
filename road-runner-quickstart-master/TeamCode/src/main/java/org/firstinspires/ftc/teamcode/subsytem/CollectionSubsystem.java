package org.firstinspires.ftc.teamcode.subsytem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class CollectionSubsystem {
    private Servo extensionServo1;
    private DcMotor intakeMotor;
    private Servo tiltServo1;
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private DepositSubsystem depositSubsystem;

    public static double intakePower = 1.0;  // Full speed intake
    public static double slowIntakePower = .75; // Reduced intake speed when detecting block

    public final double MIN_EXTENSION = 0.765;
    public final double MAX_EXTENSION = 0.3;

    public final double TILT_UP_POSITION = .3;
    public final double TILT_DOWN_POSITION = .5;

    public enum IntakeState {
        IDLE,
        EXTENDING,
        INTAKING,
        SLOWING,
        RETRACTING,
        CLOSING_CLAW
    }

    private IntakeState currentState = IntakeState.IDLE;
    private boolean objectDetected = false;

    public CollectionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, DepositSubsystem depositSubsystem) {
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();
        this.depositSubsystem = depositSubsystem;

        extensionServo1 = hardwareMap.get(Servo.class, "extensionServo1");
        extensionServo1.setDirection(Servo.Direction.REVERSE);
        tiltServo1 = hardwareMap.get(Servo.class, "tiltServo1");
        tiltServo1.setDirection(Servo.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean detectValidObject() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        boolean detectedYellow = (red >= 900 && green >= 1300);
        boolean detectedBlue = (blue >= 500 && red < 900 && green < 1300);
        boolean detectedRed = (red >= 900 && green < 1300);

        return detectedYellow || detectedBlue || detectedRed;
    }

    public void updateState(boolean extendButton) {
        boolean detected = detectValidObject();

        switch (currentState) {
            case IDLE:
                if (extendButton) {
                    currentState = IntakeState.EXTENDING;
                }
                break;

            case EXTENDING:
                if (extensionServo1.getPosition() > MAX_EXTENSION) {
                    extensionServo1.setPosition(extensionServo1.getPosition() - .015);
                    tiltServo1.setPosition(TILT_DOWN_POSITION);
                } else {
                    currentState = IntakeState.INTAKING;
                }
                break;

            case INTAKING:
                intakeMotor.setPower(intakePower);
                if (detected) {
                    objectDetected = true;
                    currentState = IntakeState.SLOWING;
                }
                break;

            case SLOWING:
                intakeMotor.setPower(slowIntakePower);
                if (!detected) {
                    currentState = IntakeState.RETRACTING;
                }
                break;

            case RETRACTING:
                intakeMotor.setPower(0);
                if (extensionServo1.getPosition() < MIN_EXTENSION) {
                    extensionServo1.setPosition(MIN_EXTENSION);
                } else {
                    tiltServo1.setPosition(TILT_UP_POSITION);
                    currentState = IntakeState.CLOSING_CLAW;
                }
                break;

            case CLOSING_CLAW:
                if (extensionServo1.getPosition() >= MIN_EXTENSION && tiltServo1.getPosition() == TILT_UP_POSITION && objectDetected) {
                    depositSubsystem.closeClaw();
                    objectDetected = false;
                    currentState = IntakeState.IDLE;
                }
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.update();
    }
}
