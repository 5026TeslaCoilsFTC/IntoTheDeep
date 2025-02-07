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
@Autonomous(name ="test", group = "Autonomous")
public class test extends LinearOpMode{
    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private enum Stage{
        preloadM,
        pushIntoZone,
        collectZone1,
        collect1M,
        collect1S,
        place1M,
        collect2M,
        collect2S,
        place2M,
        idle

    }
    Stage stage = Stage.preloadM;
    public void runOpMode() {
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);

        depositSubsystem.closeClaw();


        Pose2d intialPose = new Pose2d(10, -61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(-90)), Math.toRadians(90));
        Action preloadSpec;
        preloadSpec = preloadspecPlace.build();
        TrajectoryActionBuilder pushZone = drive.actionBuilder(new Pose2d(10, -31, Math.toRadians(-90)



                ))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(-90)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36.5, Math.toRadians(90)), Math.toRadians(0))

                .waitSeconds(.4);
        Action PushZone;
        PushZone = pushZone.build();
        TrajectoryActionBuilder pushZoneC = drive.actionBuilder(new Pose2d(49, -36.5, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -44.5), Math.toRadians(90))
                ;
        Action PushZoneC;
        PushZoneC = pushZoneC.build();
        TrajectoryActionBuilder collectSpec = drive.actionBuilder(new Pose2d(50, -45, Math.toRadians(90)))

                .waitSeconds(.5)
                .setTangent(Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(53, -50.5), Math.toRadians(90))
                .waitSeconds(.5);


        Action CollectSpec;
        CollectSpec = collectSpec.build();
        TrajectoryActionBuilder specPlace1 = drive.actionBuilder(new Pose2d(42, -50.5, Math.toRadians(90)))
                .waitSeconds(.4)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(-90)), Math.toRadians(90));
        Action SpecPlace1;
//        Action closeClaw = depositSubsystem.closeClaw();
        SpecPlace1 = specPlace1.build();
        TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(new Pose2d(10, -30, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(10, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(42,-45, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(50, -53, Math.toRadians(90)), Math.toRadians(-90));


        Action CollectSpec2;
        CollectSpec2 = collectSpec2.build();
        TrajectoryActionBuilder specPlace2 = drive.actionBuilder(new Pose2d(53, -50.5, Math.toRadians(90)))
                .waitSeconds(.4)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(5, -30, Math.toRadians(-90)), Math.toRadians(90));
        Action SpecPlace2;
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(5, -30, Math.toRadians(-90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(42, -58, Math.toRadians(-90)), Math.toRadians(90));
//        Action closeClaw = depositSubsystem.closeClaw();
        SpecPlace2 = specPlace2.build();
        Action Park;
        Park = park.build();
        waitForStart();
        depositSubsystem.closeClaw(); // Close claw
        depositSubsystem.armPlace();
        depositSubsystem.tiltPlacespec();
        ElapsedTime clawClose = new ElapsedTime();
        ElapsedTime collect = new ElapsedTime();
        while (opModeIsActive()) {
            depositSubsystem.updateTilt();
            depositSubsystem.updateSlide();

        }

    }

}

