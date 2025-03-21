//package org.firstinspires.ftc.teamcode.auto;
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
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
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
//import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
//import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
//@Config
//@Autonomous(name ="specSideAuto1Spec", group = "Autonomous")
//public class oneSpec extends LinearOpMode{
//    private DepositSubsystem depositSubsystem;
//    private DriveSubsystem driveSubsystem;
//    private CollectionSubsystem collectionSubsystem;
//    private enum Stage{
//        preloadM,
//        openClawPreload,
//        collect1M,
//        collect1S,
//        place1M,
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
//
//        Pose2d intialPose = new Pose2d(10, -61, Math.toRadians(-90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
//        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
//                .waitSeconds(.4)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(10, -32, Math.toRadians(-90)), Math.toRadians(90));
//        Action preloadSpec;
//        preloadSpec = preloadspecPlace.build();
//        TrajectoryActionBuilder collectSpec = drive.actionBuilder(drive.pose)
//
//                .setTangent(Math.toRadians(0))
//
//                .splineToLinearHeading(new Pose2d(42, -40, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(0))
//
//                .splineToLinearHeading(new Pose2d(42, -51, Math.toRadians(90)), Math.toRadians(-90));
//
//        Action CollectSpec;
//        CollectSpec = collectSpec.build();
//        TrajectoryActionBuilder specPlace1 = drive.actionBuilder(drive.pose)
//                .waitSeconds(.4)
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(-55)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(-90)), Math.toRadians(90));
//        Action SpecPlace1;
////        Action closeClaw = depositSubsystem.closeClaw();
//        SpecPlace1 = specPlace1.build();
//        depositSubsystem.armMiddle();
//        waitForStart();
//        depositSubsystem.closeClaw(); // Close claw
//        depositSubsystem.armPlace();
//        depositSubsystem.tiltPlacespec();
//        ElapsedTime clawClose = new ElapsedTime();
//        while (opModeIsActive() && !isStopRequested()) {
//            depositSubsystem.updateSlide();
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
//            switch (stage) {
//                case preloadM:
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    preloadSpec
//
//                            )
//
//
//                    );
//
//                    depositSubsystem.openClaw();
//// Open claw
//                    stage = Stage.idle;
//
//                    break;
//                case collect1M:
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    CollectSpec
//
//                            )
//
//
//                    );
//                    depositSubsystem.openClaw(); // Open claw
//                    depositSubsystem.armCollectSpec();
//                    depositSubsystem.tiltPlace();
//
//                    clawClose.reset();
//                    stage = Stage.collect1S;
//
//                    break;
//                case collect1S:
//                    if(clawClose.seconds() > .4){
//                        depositSubsystem.closeClaw();
//                    }
//                    if(clawClose.seconds()> .8){
//                        depositSubsystem.armPlace();
//                        depositSubsystem.tiltPlacespec();
//                        stage = Stage.place1M;
//                    }
//                    break;
//                case place1M:
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    SpecPlace1
//
//                            )
//
//
//                    );
//
//                    stage = Stage.idle;
//
//                    break;
//                case idle:
//                    depositSubsystem.openClaw();
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
