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
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
@Config
@Autonomous(name ="specSideAuto", group = "Autonomous")
public class specSideAuto extends LinearOpMode{
    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    public void runOpMode(){
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);
        depositSubsystem.setTilt(.5);
        depositSubsystem.closeClaw();
        depositSubsystem.tiltPlacespec();

        Pose2d intialPose = new Pose2d(-15, -61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
                .waitSeconds(.4)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( -15, -33,Math.toRadians(-90)), Math.toRadians(90));
        Action preloadSpec;
        preloadSpec = preloadspecPlace.build();
        TrajectoryActionBuilder collectSpec = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(15,-50,Math.toRadians(90)),Math.toRadians(90));

        Action CollectSpec;
        CollectSpec = collectSpec.build();
        TrajectoryActionBuilder specPlace1 = drive.actionBuilder(intialPose)
                .waitSeconds(.4)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( -15, -33,Math.toRadians(-90)), Math.toRadians(90));
        Action SpecPlace1;
//        Action closeClaw = depositSubsystem.closeClaw();
        SpecPlace1 = specPlace1.build();
        waitForStart();
        depositSubsystem.closeClaw(); // Close claw
        depositSubsystem.setTiltPlace();
        depositSubsystem.tiltPlacespec();

        Actions.runBlocking(
                new SequentialAction(
                        preloadSpec

                )



        );
        depositSubsystem.openClaw(); // Open claw
        Actions.runBlocking(
                new SequentialAction(
                        CollectSpec

                )



        );
        depositSubsystem.openClaw(); // Open claw
        depositSubsystem.setTiltSpecCollect();
        depositSubsystem.tiltPlace();
        Actions.runBlocking(
                new SequentialAction(
                        SpecPlace1

                )



        );


//        public void PlaceSpec(){
//            depositSubsystem.closeClaw();
//        }


    }
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            depositSubsystem.clawServo.setPosition(0);
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }
    public class TiltSpec implements Action{
        @Override
        public  boolean run(@NonNull TelemetryPacket packet){
            depositSubsystem.tiltPlace();
            return false;
        }
    }
    public Action tiltSpec(){
        return new TiltSpec();
    }

}

