package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SerialNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Single & Dual Servo Testing", group = "TeleOp")
public class ServoTuning extends OpMode {

    private List<Servo> servos = new ArrayList<>();
    private List<String> servoNames = new ArrayList<>();
    private int selectedServoIndex1 = 0; // Index of the first selected servo
    private int selectedServoIndex2 = 1; // Index of the second selected servo
    private double currentPosition1 = 0.5; // Position for the first servo
    private double currentPosition2 = 0.5; // Position for the second servo

    private final double INCREMENT = 0.01; // Step size for position adjustments

    private boolean dualMode = false; // Toggle between single and dual servo modes

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Populate the servo list from the hardware map
        for (Map.Entry<String, Servo> deviceName : hardwareMap.servo.entrySet()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, (SerialNumber) deviceName);
                servos.add(servo);
                servoNames.add(String.valueOf(deviceName));
                servo.setPosition(0.5); // Set default position
            } catch (Exception e) {
                telemetry.addData("Error", "Device %s is not a servo", deviceName);
            }
        }

        if (servos.isEmpty()) {
            telemetry.addData("Error", "No servos found in hardware map!");
        } else {
            telemetry.addData("Servos Found", servos.size());
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addData("Error", "No servos available for testing.");
            telemetry.update();
            return;
        }

        // Toggle between single and dual mode with gamepad1 A button
        if (gamepad1.a) {
            dualMode = !dualMode;
            sleep(200); // Debounce to prevent rapid toggling
        }

        if (dualMode) {
            // === Dual Servo Mode ===
            handleDualServoMode();
        } else {
            // === Single Servo Mode ===
            handleSingleServoMode();
        }

        // Display telemetry
        telemetry.addData("Mode", dualMode ? "Dual Servo Mode" : "Single Servo Mode");
        telemetry.update();
    }

    private void handleSingleServoMode() {
        // Change selected servo with D-pad Up/Down
        if (gamepad1.dpad_up) {
            selectedServoIndex1 = (selectedServoIndex1 + 1) % servos.size();
            sleep(200); // Prevent rapid cycling
        } else if (gamepad1.dpad_down) {
            selectedServoIndex1 = (selectedServoIndex1 - 1 + servos.size()) % servos.size();
            sleep(200); // Prevent rapid cycling
        }

        // Adjust position with bumpers
        if (gamepad1.right_bumper) {
            currentPosition1 = Math.min(1.0, currentPosition1 + INCREMENT);
        } else if (gamepad1.left_bumper) {
            currentPosition1 = Math.max(0.0, currentPosition1 - INCREMENT);
        }

        // Set position of the selected servo
        Servo selectedServo = servos.get(selectedServoIndex1);
        selectedServo.setPosition(currentPosition1);

        // Telemetry
        telemetry.addData("Selected Servo", servoNames.get(selectedServoIndex1));
        telemetry.addData("Current Position", currentPosition1);
    }

    private void handleDualServoMode() {
        // Change the first selected servo with D-pad Up/Down
        if (gamepad1.dpad_up) {
            selectedServoIndex1 = (selectedServoIndex1 + 1) % servos.size();
            sleep(200); // Prevent rapid cycling
        } else if (gamepad1.dpad_down) {
            selectedServoIndex1 = (selectedServoIndex1 - 1 + servos.size()) % servos.size();
            sleep(200); // Prevent rapid cycling
        }

        // Change the second selected servo with D-pad Left/Right
        if (gamepad1.dpad_right) {
            selectedServoIndex2 = (selectedServoIndex2 + 1) % servos.size();
            sleep(200); // Prevent rapid cycling
        } else if (gamepad1.dpad_left) {
            selectedServoIndex2 = (selectedServoIndex2 - 1 + servos.size()) % servos.size();
            sleep(200); // Prevent rapid cycling
        }

        // Adjust positions of both servos with bumpers
        if (gamepad1.right_bumper) {
            currentPosition1 = Math.min(1.0, currentPosition1 + INCREMENT);
            currentPosition2 = Math.min(1.0, currentPosition2 + INCREMENT);
        } else if (gamepad1.left_bumper) {
            currentPosition1 = Math.max(0.0, currentPosition1 - INCREMENT);
            currentPosition2 = Math.max(0.0, currentPosition2 - INCREMENT);
        }

        // Set positions of both servos
        Servo selectedServo1 = servos.get(selectedServoIndex1);
        Servo selectedServo2 = servos.get(selectedServoIndex2);

        selectedServo1.setPosition(currentPosition1);
        selectedServo2.setPosition(currentPosition2);

        // Telemetry
        telemetry.addData("Selected Servo 1", servoNames.get(selectedServoIndex1));
        telemetry.addData("Current Position 1", currentPosition1);
        telemetry.addData("Selected Servo 2", servoNames.get(selectedServoIndex2));
        telemetry.addData("Current Position 2", currentPosition2);
    }

    /**
     * Helper method to introduce a short delay for debounce
     */
    private void sleep(int milliseconds) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds) {
            // Do nothing, just wait
        }
    }
}
