package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
@Config
@Autonomous(name ="Basket Side Auto", group = "Autonomous")
public class basketSideAuto extends LinearOpMode{
    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private enum Stage{
        preloadM,
        openClawPreload,
        collect1M,
        collect1S,
        place1M,
        idle

    }
    Stage stage = Stage.preloadM;
    public void runOpMode() {
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);
        depositSubsystem.setTilt(.25);
        depositSubsystem.closeClaw();
        depositSubsystem.tiltPlacespec();

        Pose2d intialPose = new Pose2d(10, -61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
                .waitSeconds(.4)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(-90)), Math.toRadians(90));
        Action preloadSpec;
        preloadSpec = preloadspecPlace.build();

        waitForStart();
        depositSubsystem.closeClaw(); // Close claw
        depositSubsystem.setTiltPlace();
        depositSubsystem.tiltPlacespec();
        ElapsedTime clawClose = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested()) {
            depositSubsystem.updateSlide();
            switch (stage) {
                case preloadM:
                    Actions.runBlocking(
                            new SequentialAction(
                                    preloadSpec

                            )


                    );

                    depositSubsystem.openClaw();
                    depositSubsystem.setTiltCollect();
                    depositSubsystem.tiltPlace();// Open claw
                    stage = Stage.collect1M;
                    break;


                case idle:

                    break;



//        public void PlaceSpec(){
//            depositSubsystem.closeClaw();
//        }

            }
        }
    }

}

