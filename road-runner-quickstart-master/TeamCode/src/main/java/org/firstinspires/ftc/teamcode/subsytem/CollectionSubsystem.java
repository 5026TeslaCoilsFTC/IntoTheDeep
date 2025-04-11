package org.firstinspires.ftc.teamcode.subsytem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.colorClassifierTest;
@Config
public class CollectionSubsystem {
    public Servo extensionServo1;
    private DcMotor intakeMotor;
    public Servo tiltServo1;
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private DepositSubsystem depositSubsystem;
    private final colorClassifierTest colorClassifier;
    public static double intakePower = 1.0;  // Full speed intake
    public static double slowIntakePower = .3; // Reduced intake speed when detecting block

    public final double MIN_EXTENSION = .38;
    public final double MAX_EXTENSION = 0;

    public final double TILT_UP_POSITION = .31;
    public final double TILT_DOWN_POSITION = .53;
    public static boolean clawClosed = false;
    private ElapsedTime clawDelayTimer = new ElapsedTime();
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
        colorClassifier = new colorClassifierTest(colorSensor);
    }
    public enum AllianceColor {
        BLUE,
        RED,
        BOTH // Accepts both red and blue (for tuning or testing)
    }

    public static AllianceColor allianceColor = AllianceColor.BOTH;

    public boolean detectValidObject() {
        colorClassifierTest.DetectedColor detectedColor = colorClassifier.getColor();

        switch (detectedColor) {
            case YELLOW:
                return true;
            case BLUE:
                return (allianceColor == AllianceColor.BLUE || allianceColor == AllianceColor.BOTH);
            case RED:
                return (allianceColor == AllianceColor.RED || allianceColor == AllianceColor.BOTH);
            default:
                return false;
        }
    }



    public void updateState(boolean extendButton) {
        boolean detected = detectValidObject();

        switch (currentState) {
            case IDLE:
                intakeMotor.setPower(0);
                if (extendButton) {
                    currentState = IntakeState.EXTENDING;
                }
                break;

            case EXTENDING:
                if (extensionServo1.getPosition() > MAX_EXTENSION) {
                    extensionServo1.setPosition(MAX_EXTENSION);
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
                tiltServo1.setPosition(TILT_UP_POSITION);
                extensionServo1.setPosition(MIN_EXTENSION);
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

                }
                clawDelayTimer.reset();
                currentState = IntakeState.CLOSING_CLAW;
                break;
            case CLOSING_CLAW:
                if (clawDelayTimer.seconds() >= 0.5) {
                    depositSubsystem.closeClaw();                        clawClosed = true;
                    currentState = IntakeState.IDLE;
                }
                break;


        }

        telemetry.addData("Red: ",colorSensor.red());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Yellow, ", colorSensor.green());
        telemetry.addData("State", currentState);
        telemetry.addData("Claw Close Timer: ", clawDelayTimer.seconds());
        telemetry.update();
    }
    public void reverseIntake(){
        intakeMotor.setPower(-1);
    }
    public void resetState(){
        currentState = IntakeState.IDLE;
        tiltServo1.setPosition(TILT_UP_POSITION);
        extensionServo1.setPosition(MIN_EXTENSION);
    }
}
