//package org.firstinspires.ftc.teamcode.auto;
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
//import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
//import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
//@Config
//@Autonomous(name ="specSideAuto3  SpecPush", group = "Autonomous")
//public class speciSideAutoPushCycle3 extends LinearOpMode{
//    private DepositSubsystem depositSubsystem;
//    private DriveSubsystem driveSubsystem;
//    private CollectionSubsystem collectionSubsystem;
//    private enum Stage{
//        preloadM,
//        pushIntoZone,
//        place1M,
//        collect1M,
//        collect1S,
//
//        collect2M,
//        collect2S,
//        place2M,
//
//
//
//        idle
//
//    }
//    Stage stage = Stage.preloadM;
//    public void runOpMode() {
//        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
//        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
//        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);
//
//        depositSubsystem.closeClaw();
//
//        Pose2d intialPose = new Pose2d(10, -61, Math.toRadians(-90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
//        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(-90)), Math.toRadians(90));
//        Action preloadSpec;
//        preloadSpec = preloadspecPlace.build();
//        TrajectoryActionBuilder backUp = drive.actionBuilder(new Pose2d(10, -30, Math.toRadians(-90)
//
//
//
//                ))
//                .splineToLinearHeading(new Pose2d(10,-40, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(25, -42, Math.toRadians(0)), Math.toRadians(-90));
//
//        TrajectoryActionBuilder pushZone = drive.actionBuilder(new Pose2d(25, -42, Math.toRadians(0)
//
//
//
//                ))
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)), Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(45,-50, Math.toRadians(0)), Math.toRadians(-90))
//
//
//                ;
//
//
//        Action BackUp;
//        BackUp = backUp.build();
//        Action PushZone;
//        PushZone = pushZone.build();
//        TrajectoryActionBuilder collectSpec = drive.actionBuilder(new Pose2d(45, -50, Math.toRadians(0)))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,-42, Math.toRadians(90)), Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(54, -48, Math.toRadians(90)), Math.toRadians(-90))
//                ;
//
//
//
//        Action CollectSpec;
//        CollectSpec = collectSpec.build();
//        TrajectoryActionBuilder specPlace1 = drive.actionBuilder(new Pose2d(54, -48.5, Math.toRadians(-90)))
//                .splineToLinearHeading(new Pose2d(54, -44, Math.toRadians( -90)), Math.toRadians(-90))
//                .waitSeconds(.4)
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(0, -32, Math.toRadians(-90)), Math.toRadians(90));
//        Action SpecPlace1;
////        Action closeClaw = depositSubsystem.closeClaw();
//        SpecPlace1 = specPlace1.build();
//        TrajectoryActionBuilder collectSpec2 = drive.actionBuilder(new Pose2d(10, -30, Math.toRadians(-90)))
//                .splineToLinearHeading(new Pose2d(10, -45, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(42,-45, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(180))
//
//                .splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(0))
//
//                .splineToLinearHeading(new Pose2d(50, -53, Math.toRadians(90)), Math.toRadians(-90));
//
//
//        Action CollectSpec2;
//        CollectSpec2 = collectSpec2.build();
//        TrajectoryActionBuilder specPlace2 = drive.actionBuilder(new Pose2d(53, -50.5, Math.toRadians(90)))
//                .waitSeconds(.4)
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(5, -30, Math.toRadians(-90)), Math.toRadians(90));
//        Action SpecPlace2;
//        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(5, -30, Math.toRadians(-90)))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(42, -58, Math.toRadians(-90)), Math.toRadians(90));
////        Action closeClaw = depositSubsystem.closeClaw();
//        SpecPlace2 = specPlace2.build();
//
//        Action Park;
//        Park = park.build();
//        depositSubsystem.armMiddle();
//        waitForStart();
//        depositSubsystem.closeClaw();
//        depositSubsystem.updateSlide();// Close claw
//        depositSubsystem.tiltPlacespec();
//        ElapsedTime clawClose = new ElapsedTime();
//        ElapsedTime collect = new ElapsedTime();
//        while (opModeIsActive() && !isStopRequested()) {
//
//
//
//// Start tilt control in a separate thread (non-blocking)
//
//
//            if(depositSubsystem.liftMotor1.getCurrentPosition()>500 && depositSubsystem.liftMotor1.getCurrentPosition()<750){
//                // Close claw
//                depositSubsystem.armPlace();
//
//            }
//            else if(depositSubsystem.liftMotor1.getCurrentPosition()> 1000){
//                depositSubsystem.tiltPlaceSpec();
//            }
//            else if(depositSubsystem.liftMotor1.getCurrentPosition()> 250 && depositSubsystem.liftMotor1.getCurrentPosition()<450){
//                depositSubsystem.armCollect();
//                depositSubsystem.tiltPlacec();
//
//            }
//
//            telemetry.addData("Claw Close: ", clawClose.seconds());
//
//            telemetry.update();
//            switch (stage) {
//                case preloadM:
//                    depositSubsystem.armPlace();
//                    depositSubsystem.tiltPlacespec();
//                    Actions.runBlocking(
//                            new ParallelAction(
//
//                                    preloadSpec
//
//                            )
//
//
//                    );
//                    depositSubsystem.openClaw();
//// Open claw
//                    stage = Stage.pushIntoZone;
//
//                    break;
//                case pushIntoZone:
//                    //collectionSubsystem.extend();
//
//                    Actions.runBlocking(
//                            new ParallelAction(
//                                    BackUp
//
//                            )
//
//
//                    );
//                    depositSubsystem.armMiddle();
//                    depositSubsystem.tiltPlacec();
//                    Actions.runBlocking(
//                            new ParallelAction(
//
//                                    PushZone
//                            )
//                    );
//                    collect.reset();
//                    clawClose.reset();
//                    stage = Stage.collect1M;
//                    break;
//
//
//                case collect1M:
//                    depositSubsystem.armCollectSpec();
//
//                    Actions.runBlocking(
//                            new ParallelAction(
//
//
//                                    CollectSpec
//                            )
//                    );
//
//
//                    depositSubsystem.openClaw();
//                    clawClose.reset();
//                    stage = Stage.collect1S;
//
//                    break;
//                case collect1S:
//                    if(clawClose.seconds()> .8 && clawClose.seconds() <1){
//                        depositSubsystem.tiltPlace();
//                    }
//                    if(clawClose.seconds() > 1.4){
//                        depositSubsystem.closeClaw();
//                    }
//                    if(clawClose.seconds()> 2){
//                        depositSubsystem.armPlace();
//                        depositSubsystem.tiltPlacespec();
//                        stage = Stage.place1M;
//                    }
//                    break;
//                case place1M:
//                    Actions.runBlocking(
//                            new ParallelAction(
//
//                                    SpecPlace1
//
//                            )
//
//
//                    );
//
//                    stage = Stage.collect2M;
//
//                    break;
//                case collect2M:
//
//                    depositSubsystem.openClaw();
//                    depositSubsystem.armCollectSpec();
//                    depositSubsystem.tiltPlace();
//                    Actions.runBlocking(
//                            new ParallelAction(
//                                    CollectSpec2
//                            )
//                    );
//                    clawClose.reset();
//                    stage = Stage.idle;
//                    break;
//                case collect2S:
//                    if(clawClose.seconds()>.4){
//                        depositSubsystem.closeClaw();
//                    }
//
//                    if(clawClose.seconds()> 1.5) {// Close claw
//                        depositSubsystem.armPlace();
//                        depositSubsystem.tiltPlacespec();
//                        stage = Stage.place2M;
//                    }
//                    break;
//                case place2M:
//
//
//                    Actions.runBlocking(
//                            new ParallelAction(
//
//                                    SpecPlace2
//                            )
//                    );
//                    stage = Stage.idle;
//                    break;
//
//                case idle:
//                    //  depositSubsystem.openClaw();
//
//                    break;
//
//
//
////        public void PlaceSpec(){
////            depositSubsystem.closeClaw();
////        }
//
//            }
//        }
//    }
//
//}
//
