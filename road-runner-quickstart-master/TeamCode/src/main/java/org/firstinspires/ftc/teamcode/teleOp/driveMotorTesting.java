package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Motor Testing", group = "TeleOp")
public class driveMotorTesting extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private int selectedMotor = 0; // Index of the currently selected motor
    private final String[] motorNames = {"fl", "bl", "fr", "br"};
    private DcMotor[] motors;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Initialize the motors
        try {
            leftFront = hardwareMap.get(DcMotor.class, "fl");
            leftRear = hardwareMap.get(DcMotor.class, "bl");
            rightFront = hardwareMap.get(DcMotor.class, "fr");
            rightRear = hardwareMap.get(DcMotor.class, "br");
        } catch (Exception e) {
            telemetry.addData("Error", "Check motor names in hardware map!");
        }

        // Store motors in an array for easy selection
        motors = new DcMotor[]{leftFront, leftRear, rightFront, rightRear};

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Ensure motors are initialized
        if (motors.length == 0 || motors[0] == null) {
            telemetry.addData("Error", "Motors not properly initialized!");
            telemetry.update();
            return;
        }

        // Select motor with D-pad
        if (gamepad1.dpad_up) {
            selectedMotor = (selectedMotor + 1) % motors.length;
        } else if (gamepad1.dpad_down) {
            selectedMotor = (selectedMotor - 1 + motors.length) % motors.length;
        }

        // Control the selected motor with the right stick
        double power = -gamepad1.right_stick_y; // Up for forward, down for backward
        motors[selectedMotor].setPower(power);

        // Stop all other motors
        for (int i = 0; i < motors.length; i++) {
            if (i != selectedMotor) {
                motors[i].setPower(0);
            }
        }

        // Telemetry
        telemetry.addData("Selected Motor", motorNames[selectedMotor]);
        telemetry.addData("Current Power", power);
        telemetry.addData("Instructions", "Use D-pad to change motor, right stick to control power.");
        telemetry.update();
    }
}
