package org.firstinspires.ftc.teamcode.subsytem;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem {
    public DcMotor leftFront, leftRear, rightFront, rightRear;
    private Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotor.class, "fl");
        leftRear = hardwareMap.get(DcMotor.class, "bl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        rightRear = hardwareMap.get(DcMotor.class, "br");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void drive(double yInput, double lxInput, double rxInput, double speed) {
        double y = yInput * speed; // Remember, this is reversed!
        double lx = lxInput * speed; // Counteract imperfect strafing
        double rx = rxInput * speed;

        double denominator = Math.max(Math.abs(y) + Math.abs(lx) + Math.abs(rx), 1);
        double frontLeftPower = (y + lx + rx) / denominator;
        double backLeftPower = (y - lx + rx) / denominator;
        double frontRightPower = (y - lx - rx) / denominator;
        double backRightPower = (y + lx - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        telemetry.addData("Drive", "LF: %.2f, LR: %.2f, RF: %.2f, RR: %.2f",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void driveForward(double s){
        leftFront.setPower(s);
        leftRear.setPower(s);
        rightFront.setPower(s);
        rightRear.setPower(s);
    }

    public void driveLeft(double s){
        leftFront.setPower(-s);
        leftRear.setPower(s);
        rightFront.setPower(s);
        rightRear.setPower(-s);
    }

    public void driveRight(double s){
        leftFront.setPower(s);
        leftRear.setPower(-s);
        rightFront.setPower(-s);
        rightRear.setPower(s);
    }

    public void driveStop(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

}
