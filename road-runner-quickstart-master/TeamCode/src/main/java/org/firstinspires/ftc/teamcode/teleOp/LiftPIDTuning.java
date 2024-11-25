//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import org.firstinspires.ftc.teamcode.dashboard.FtcDashboard;
//
//@TeleOp(name = "Lift PID Tuning with FF", group = "TeleOp")
//public class LiftPIDTuning extends OpMode {
//
//    private DcMotor liftMotor1, liftMotor2;
//    private PIDController pidController;
//
//    private double p = 0.01, i = 0.0, d = 0.0; // PID coefficients
//    private double f = 0.1; // Feedforward coefficient
//    private double targetPosition = 0.0; // Target position in encoder ticks
//    private double ticksInDegree = 10.0; // Ticks per degree of rotation
//
//    private FtcDashboard dashboard;
//
//    @Override
//    public void init() {
//        // Initialize motors
//        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
//        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
//
//        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Initialize PID controller
//        pidController = new PIDController(p, i, d);
//
//        // Initialize FTC Dashboard
//        dashboard = FtcDashboard.getInstance();
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void loop() {
//        // Update PID coefficients from dashboard
//        updatePIDCoefficientsFromDashboard();
//
//        // Use D-pad to adjust the target position
//        if (gamepad1.dpad_up) {
//            targetPosition += 50; // Increment target
//        } else if (gamepad1.dpad_down) {
//            targetPosition -= 50; // Decrement target
//        }
//
//        // Get the current position of the lift
//        int currentPosition = (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition()) / 2;
//
//        // Set the target position in the PID controller
//        pidController.setSetPoint(targetPosition);
//
//        // Calculate PID output
//        double pidOutput = pidController.calculate(currentPosition);
//
//        // Calculate feedforward
//        double feedforward = Math.cos(Math.toRadians(targetPosition / ticksInDegree)) * f;
//
//        // Combine PID output and feedforward
//        double power = pidOutput + feedforward;
//
//        // Clamp the power to the range [-1, 1]
//        power = Math.max(-1.0, Math.min(1.0, power));
//
//        // Apply power to the motors
//        liftMotor1.setPower(power);
//        liftMotor2.setPower(power);
//
//        // Display telemetry on the dashboard
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//        dashboardTelemetry.addData("Target Position", targetPosition);
//        dashboardTelemetry.addData("Current Position", currentPosition);
//        dashboardTelemetry.addData("PID Output", pidOutput);
//        dashboardTelemetry.addData("Feedforward", feedforward);
//        dashboardTelemetry.addData("Total Power", power);
//        dashboardTelemetry.update();
//
//        // Display telemetry on the driver station
//        telemetry.addData("Target Position", targetPosition);
//        telemetry.addData("Current Position", currentPosition);
//        telemetry.addData("PID Output", pidOutput);
//        telemetry.addData("Feedforward", feedforward);
//        telemetry.addData("Total Power", power);
//        telemetry.update();
//    }
//
//    /**
//     * Updates the PID coefficients from the FTC Dashboard.
//     */
//    private void updatePIDCoefficientsFromDashboard() {
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//
//        // Retrieve values from the dashboard or use current ones if not updated
//        p = dashboardTelemetry.getDouble("P Coefficient", p);
//        i = dashboardTelemetry.getDouble("I Coefficient", i);
//        d = dashboardTelemetry.getDouble("D Coefficient", d);
//        f = dashboardTelemetry.getDouble("Feedforward", f);
//
//        // Update the PID controller with the new coefficients
//        pidController.setPID(p, i, d);
//    }
//}
