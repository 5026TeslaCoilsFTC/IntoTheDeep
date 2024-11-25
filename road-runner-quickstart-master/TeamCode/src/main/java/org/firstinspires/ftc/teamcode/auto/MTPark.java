package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;

@Autonomous(name = "Park Only", group = "TeleOp")
public class MTPark extends LinearOpMode {

    private DriveSubsystem robot;

    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();

        robot.driveForward(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.driveStop();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Pause 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.driveStop();

    }
}
