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
@Autonomous(name ="specSideAuto3Spec", group = "Autonomous")
public class specSideAutoCycles extends LinearOpMode{
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

        depositSubsystem.tiltPlacespec();
        ElapsedTime clawClose = new ElapsedTime();
        ElapsedTime collect = new ElapsedTime();
        while (opModeIsActive()) {

            depositSubsystem.updateSlide();
            telemetry.addData("Claw Close: ", clawClose.seconds());
            telemetry.update();
            switch (stage) {
                case preloadM:
                    depositSubsystem.armPlace();

                    Actions.runBlocking(
                            new SequentialAction(
                                    preloadSpec

                            )


                    );
                    depositSubsystem.openClaw();
// Open claw
                    stage = Stage.pushIntoZone;

                    break;
                case pushIntoZone:


                    //collectionSubsystem.extend();
                    collectionSubsystem.tiltRetract();
                    collectionSubsystem.collect();
                    Actions.runBlocking(
                            new SequentialAction(
                                    PushZone

                            )


                    );

                    collect.reset();
                    clawClose.reset();
                    stage = Stage.collectZone1;
                    break;

                case collectZone1:
                    if(collect.seconds()<.5){
                       collectionSubsystem.stopCollection();
                       collectionSubsystem.tiltCollect();
                      // collectionSubsystem.retractFull();
                    }
                    else if(collect.seconds()>.5 && collect.seconds()<.75){
                        collectionSubsystem.reverseCollection();
                        depositSubsystem.armCollect();
                        depositSubsystem.tiltPlacec();
                        depositSubsystem.openClaw();

                    }
                    if(collect.seconds() >1.5){
                        collectionSubsystem.stopCollection();
                        depositSubsystem.closeClaw();

                    }
                    if(collect.seconds()>2&& collect.seconds()<2.25){
                        depositSubsystem.setTargetSlide(300);
                        depositSubsystem.armCollectSpec();

                    }
                    else if(collect.seconds()>3.25 && collect.seconds()<3.5){
                        depositSubsystem.tiltPlace();
                        depositSubsystem.setTargetSlide(0);

                        stage = Stage.collect1M;
                }
                    break;
                case collect1M:
                    Actions.runBlocking(
                            PushZoneC
                    );
                    depositSubsystem.openClaw();

                    Actions.runBlocking(
                            CollectSpec
                    );

                    clawClose.reset();
                    stage = Stage.collect1S;

                    break;
                case collect1S:
                    if(clawClose.seconds() > .4){
                        depositSubsystem.closeClaw();
                    }
                    if(clawClose.seconds()> .8){
                        depositSubsystem.armPlace();
                        depositSubsystem.tiltPlacespec();
                        stage = Stage.place1M;
                    }
                    break;
                case place1M:
                    Actions.runBlocking(
                            new SequentialAction(
                                    SpecPlace1

                            )


                    );

                    stage = Stage.collect2M;

                    break;
                case collect2M:

                    depositSubsystem.openClaw();
                    depositSubsystem.armCollectSpec();
                    depositSubsystem.tiltPlace();
                    Actions.runBlocking(
                            new SequentialAction(
                                    CollectSpec2
                            )
                    );
                    clawClose.reset();
                    stage = Stage.collect2S;
                    break;
                case collect2S:
                    if(clawClose.seconds()>.4){
                        depositSubsystem.closeClaw();
                    }

                    if(clawClose.seconds()> 1.5) {// Close claw
                        depositSubsystem.armPlace();
                        depositSubsystem.tiltPlacespec();
                        stage = Stage.place2M;
                    }
                    break;
                case place2M:


                    Actions.runBlocking(
                            new SequentialAction(
                                    SpecPlace2
                            )
                    );
                    stage = Stage.idle;
                    break;

                case idle:
                    depositSubsystem.openClaw();
                    Actions.runBlocking(
                            new SequentialAction(Park)
                    );
                    break;



//        public void PlaceSpec(){
//            depositSubsystem.closeClaw();
//        }

            }

        }

    }

}

