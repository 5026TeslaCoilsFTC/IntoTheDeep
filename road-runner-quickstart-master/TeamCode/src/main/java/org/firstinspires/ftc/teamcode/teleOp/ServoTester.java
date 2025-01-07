package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Servo Tester with On/Off (PWM)", group = "Tools")
public class ServoTester extends OpMode {

    private List<Servo> servos = new ArrayList<>();
    private List<String> servoNames = new ArrayList<>();
    private List<Boolean> servoActiveStates = new ArrayList<>(); // Tracks whether each servo is active
    private int selectedServoIndex = 0;
    private double servoPosition = 0.5; // Default position

    private boolean bPressedLast = false; // Track Button B press for toggling

    @Override
    public void init() {
        // Discover all servos connected to the hardware map
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            try {
                servos.add(entry.getValue());
                servoNames.add(entry.getKey());
                servoActiveStates.add(true); // Initialize all servos as active
                telemetry.addData("Discovered Servo", entry.getKey());
            } catch (Exception e) {
                telemetry.addData("Error Initializing Servo", entry.getKey());
            }
        }

        if (servos.isEmpty()) {
            telemetry.addLine("No servos found! Please connect servos.");
        } else {
            telemetry.addData("Total Servos", servos.size());
            telemetry.addLine("Use D-Pad to select and adjust servos.");
            telemetry.addLine("Press B to toggle servo On/Off.");
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addLine("No servos found! Please connect servos.");
            telemetry.update();
            return;
        }

        // Select servo with D-Pad left/right
        if (gamepad1.dpad_right) {
            selectedServoIndex = (selectedServoIndex + 1) % servos.size();
            sleep(200); // Prevent multiple selections from a single press
        } else if (gamepad1.dpad_left) {
            selectedServoIndex = (selectedServoIndex - 1 + servos.size()) % servos.size();
            sleep(200); // Prevent multiple selections from a single press
        }

        // Toggle the active state of the selected servo with Button B
        if (gamepad1.b && !bPressedLast) {
            boolean currentState = servoActiveStates.get(selectedServoIndex);
            servoActiveStates.set(selectedServoIndex, !currentState); // Toggle active state
            Servo selectedServo = servos.get(selectedServoIndex);
            if (!currentState) {
                selectedServo.getController().pwmEnable(); // Turn the servo ON
            } else {
                selectedServo.getController().pwmDisable(); // Turn the servo OFF
            }
            bPressedLast = true; // Record the press
        } else if (!gamepad1.b) {
            bPressedLast = false; // Reset when button is released
        }

        // Adjust servo position if active
        if (servoActiveStates.get(selectedServoIndex)) {
            if (gamepad1.right_stick_y != 0) {
                servoPosition += gamepad1.right_stick_y * -0.01; // Reverse Y-axis
                servoPosition = Math.max(0.0, Math.min(1.0, servoPosition)); // Clamp to [0, 1]
            }

            // Set the selected servo position
            Servo selectedServo = servos.get(selectedServoIndex);
            selectedServo.setPosition(servoPosition);
        }

        // Telemetry feedback
        telemetry.addData("Selected Servo", servoNames.get(selectedServoIndex));
        telemetry.addData("Servo Index", selectedServoIndex);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("Servo Active", servoActiveStates.get(selectedServoIndex) ? "On" : "Off");
        telemetry.addLine("Use D-Pad (left/right) to select servos.");
        telemetry.addLine("Use Right Stick (up/down) to adjust position.");
        telemetry.addLine("Press B to toggle servo On/Off.");
        telemetry.update();
    }

    private void sleep(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // Do nothing
        }
    }
}