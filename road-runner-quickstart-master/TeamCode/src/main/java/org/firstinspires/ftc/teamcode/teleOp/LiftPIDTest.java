package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "Lift PIDF Tuning", group = "TeleOp")
public class LiftPIDTest extends OpMode {

    private DcMotor liftMotor1, liftMotor2;
    private PIDFController pidfController;

    // PIDF coefficients
    public static double p = 0.00, i = 0.0, d = 0.0, f = 0.1;
    public static double targetPosition = 0.0; // Target lift position (in encoder ticks)

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize motors
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        // Initialize PIDF controller
        pidfController = new PIDFController(p, i, d, f);

        // Connect to FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Update PIDF coefficients from FTC Dashboard
        updatePIDFCoefficientsFromDashboard();

        // Adjust target position with D-pad
        if (gamepad1.dpad_up) {
            targetPosition += 50; // Increase target
        } else if (gamepad1.dpad_down) {
            targetPosition -= 50; // Decrease target
        }

        // Get the current lift position
        int currentPosition = (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition()) / 2;

        // Set the target position for the PIDF controller
        pidfController.setSetPoint(targetPosition);

        // Calculate motor power using PIDF
        double power = pidfController.calculate(currentPosition);

        // Apply power to lift motors
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        // Send telemetry to FTC Dashboard
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Target Position", targetPosition);
        dashboardTelemetry.addData("Current Position", currentPosition);
        dashboardTelemetry.addData("Motor Power", power);
        dashboardTelemetry.update();

        // Send telemetry to Driver Station
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }

    /**
     * Updates PIDF coefficients from the FTC Dashboard.
     */
    private void updatePIDFCoefficientsFromDashboard() {
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Retrieve values from the dashboard or use current ones if not updated
//        p = dashboardTelemetry.getDouble("P Coefficient", p);
//        i = dashboardTelemetry.getDouble("I Coefficient", i);
//        d = dashboardTelemetry.getDouble("D Coefficient", d);
//        f = dashboardTelemetry.getDouble("F Coefficient", f);

        // Update the PIDF controller with the new coefficients
        pidfController.setPIDF(p, i, d, f);
    }
}
