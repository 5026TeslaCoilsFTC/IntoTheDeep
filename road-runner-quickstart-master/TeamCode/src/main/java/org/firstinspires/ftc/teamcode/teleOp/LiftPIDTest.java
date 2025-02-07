package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "Lift PIDF Tuning", group = "TeleOp")
public class LiftPIDTest extends OpMode {

    private DcMotor tiltMotor;
    private PIDController pidController;

    // PIDF coefficients
    public static double p = 0.01, i = 0.0, d = 0.0006, f = 0.13;
    public static double tiltMultiplier = 1;
    double ticks_in_degree = 751.83/180;
    public static double targetPosition = 0.0; // Target lift position (in encoder ticks)

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize motors

        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        tiltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        // Initialize PIDF controller
        pidController = new PIDController(p, i, d);

        // Connect to FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Update PIDF coefficients from FTC Dashboard
        updatePIDFCoefficientsFromDashboard();

        int tiltPos = tiltMotor.getCurrentPosition();
        double pid = pidController.calculate(tiltPos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition/ ticks_in_degree))*f;
        double power = pid +ff;
        // Adjust target position with D-pad
        if (gamepad1.dpad_up) {
            targetPosition += 50; // Increase target
        } else if (gamepad1.dpad_down) {
            targetPosition -= 50; // Decrease target
        }

        // Get the current lift position


        // Set the target position for the PIDF controller


        // Calculate motor power using PIDF


        // Apply power to lift motors
tiltMotor.setPower(power* tiltMultiplier);

        // Send telemetry to FTC Dashboard
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Target Position", targetPosition);
        dashboardTelemetry.addData("Current Position", tiltPos);
        dashboardTelemetry.addData("Motor Power", power);
        dashboardTelemetry.update();

        // Send telemetry to Driver Station
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", tiltPos);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }

    /**
     * Updates PIDF coefficients from the FTC Dashboard.
     */
    private void updatePIDFCoefficientsFromDashboard() {
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

//         Retrieve values from the dashboard or use current ones if not updated
//        p = dashboardTelemetry.getDouble("P Coefficient", p);
//        i = dashboardTelemetry.getDouble("I Coefficient", i);
//        d = dashboardTelemetry.getDouble("D Coefficient", d);
//        f = dashboardTelemetry.getDouble("F Coefficient", f);

        // Update the PIDF controller with the new coefficients
        pidController.setPID(p, i, d);
    }
}
