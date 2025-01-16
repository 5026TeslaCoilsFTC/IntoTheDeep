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
        place1S,
        collect2M,
        collect2S,
        place2M,
        place2S,
        end,
        idle

    }
    Stage stage = Stage.preloadM;
    public void runOpMode() {
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);
        depositSubsystem.setTilt(.27);
        depositSubsystem.closeClaw();
        depositSubsystem.tiltPlacespec();

        Pose2d intialPose = new Pose2d(-15, -61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
                .waitSeconds(.8)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-15, -32, Math.toRadians(-90)), Math.toRadians(90));
        TrajectoryActionBuilder collect1 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-10,-50,Math.toRadians(-90)),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49,-32,Math.toRadians(110)),Math.toRadians(180));
        TrajectoryActionBuilder place1 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58,-55,Math.toRadians(45)),Math.toRadians(-90))

                ;
        TrajectoryActionBuilder collect2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-56,-32,Math.toRadians(110)),Math.toRadians(180));
        TrajectoryActionBuilder place2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58,-57,Math.toRadians(45)),Math.toRadians(-90))

                ;
        TrajectoryActionBuilder backUp = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56,-54,Math.toRadians(45)),Math.toRadians(-90))
                ;
        Action Place1;
        Place1 = place1.build();
        Action Collect1;
        Collect1 = collect1.build();
        Action Place2;
        Place2 = place2.build();
        Action Collect2;
        Collect2 = collect2.build();
        Action preloadSpec;
        preloadSpec = preloadspecPlace.build();
        Action End;
        End = backUp.build();

        waitForStart();
        depositSubsystem.closeClaw(); // Close claw
        depositSubsystem.setTiltPlace();
        depositSubsystem.tiltPlacespec();
        ElapsedTime closeClaw = new ElapsedTime();
        ElapsedTime collect = new ElapsedTime();
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
//                    depositSubsystem.setTiltCollect();
//                    depositSubsystem.tiltPlacec();
                    collectionSubsystem.tiltRetract();
                    collectionSubsystem.collect();
                    stage = Stage.collect1M;
                    break;
                case collect1M:
                    Actions.runBlocking(
                            new SequentialAction(
                                    Collect1

                            )


                    );

                    collect.reset();
                    stage =Stage.collect1S;
                    break;
                case collect1S:
                    if(collect.seconds()> .5&& collect.seconds()<1){
                        collectionSubsystem.stopCollection();
                        collectionSubsystem.tiltCollect();

                    }
                    if (collect.seconds()>1&& collect.seconds()<1.25){
                        collectionSubsystem.reverseCollection();
                        depositSubsystem.setTiltCollect();
                        depositSubsystem.tiltPlacec();
                        depositSubsystem.openClaw();
                    }
                    if (collect.seconds()>1.5) {
                        collectionSubsystem.stopCollection();

                    }
                    if(collect.seconds()>1.75){
                        depositSubsystem.closeClaw();

                        stage = Stage.place1M;
                    }
                    break;
                case place1M:
                    depositSubsystem.setTargetSlide(3300);
                    Actions.runBlocking(
                            new SequentialAction(
                                    Place1

                            )


                    );
                    closeClaw.reset();
                    stage =Stage.place1S;
                    break;
                case place1S:
                    if(closeClaw.seconds()> 3) {

                        depositSubsystem.openClaw();

                    }
                    if(closeClaw.seconds()>3.5){
                        stage = Stage.end;// Close claw
                    }
                    if(closeClaw.seconds()> 2) {
                        depositSubsystem.setTiltPlace();
                        depositSubsystem.tiltPlaceSpec();

                    }
                    break;


//                case collect2M:
//                    collectionSubsystem.tiltRetract();
//                    collectionSubsystem.collect();
//                    depositSubsystem.setTargetSlide(0);
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    Collect2
//
//                            )
//
//
//                    );
//
//                    collect.reset();
//                    stage =Stage.idle;
//                    break;
//                case collect2S:
//                    if(collect.seconds()> .5&& collect.seconds()<1){
//                        collectionSubsystem.stopCollection();
//                        collectionSubsystem.tiltCollect();
//
//                    }
//                    if (collect.seconds()>1&& collect.seconds()<1.25){
//                        collectionSubsystem.reverseCollection();
//                        depositSubsystem.setTiltCollect();
//                        depositSubsystem.tiltPlacec();
//                        depositSubsystem.openClaw();
//                    }
//                    if (collect.seconds()>1.5) {
//                        collectionSubsystem.stopCollection();
//
//                    }
//                    if(collect.seconds()>1.75){
//                        depositSubsystem.closeClaw();
//
//                        stage = Stage.place2M;
//                    }
//                    break;
//                case place2M:
//                    depositSubsystem.setTargetSlide(3300);
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    Place2
//
//                            )
//
//
//                    );
//                    closeClaw.reset();
//                    stage =Stage.place2S;
//                    break;
//                case place2S:
//                    if(closeClaw.seconds()> 3) {
//
//                        depositSubsystem.openClaw();
//                        stage = Stage.idle;
//                    }// Close claw
//                    if(closeClaw.seconds()> 2) {
//                        depositSubsystem.setTiltPlace();
//                        depositSubsystem.tiltPlaceSpec();
//
//                    }
//                    break;
                case end:
                    Actions.runBlocking(
                            new SequentialAction(
                                    End

                            )


                    );
                    depositSubsystem.setTargetSlide(0);
                    stage = Stage.idle;
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

