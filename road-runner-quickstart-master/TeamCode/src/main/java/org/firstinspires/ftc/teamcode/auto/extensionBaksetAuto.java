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
@Autonomous(name ="Extension Basket Side Auto", group = "Autonomous")
public class extensionBaksetAuto extends LinearOpMode{
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
        collect3M,
        collect3S,
        end,
        idle

    }
    Stage stage = Stage.preloadM;
    public void runOpMode() {
        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry);

        depositSubsystem.closeClaw();


        Pose2d intialPose = new Pose2d(-39, -61, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        TrajectoryActionBuilder preloadspecPlace = drive.actionBuilder(intialPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53, -55, Math.toRadians( 45)), Math.toRadians(180));
        TrajectoryActionBuilder collect1 = drive.actionBuilder(new Pose2d(-53, -55, Math.toRadians(45)))

                .splineToLinearHeading(new Pose2d(-48,-50, Math.toRadians(92)), Math.toRadians( 90));
        TrajectoryActionBuilder place1 = drive.actionBuilder(new Pose2d(-48,-50,Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-51,-54,Math.toRadians(45)), Math.toRadians(-135))

                ;
        TrajectoryActionBuilder collect2 = drive.actionBuilder(new Pose2d(-51,-54,Math.toRadians(45)))
                .setTangent( Math.toRadians(45))

                .splineToLinearHeading(new Pose2d(-57,-50, Math.toRadians(90)), Math.toRadians( 90));
        TrajectoryActionBuilder place2 = drive.actionBuilder(new Pose2d(-57,-50,Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55,-60,Math.toRadians(45)), Math.toRadians(-135))

                ;
        TrajectoryActionBuilder collect3 = drive.actionBuilder(new Pose2d(-65,-42,Math.toRadians(45)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60, -30, Math.toRadians(180)), Math.toRadians(180));
        TrajectoryActionBuilder backUp = drive.actionBuilder(new Pose2d(-66,-42,Math.toRadians(45)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56,-38,Math.toRadians(45)),Math.toRadians(-90))
                ;
        Action Place1;
        Place1 = place1.build();
        Action Collect1;
        Collect1 = collect1.build();
        Action Place2;
        Place2 = place2.build();
        Action Collect2;
        Collect2 = collect2.build();
        Action Collect3;
        Collect3 = collect3.build();
        Action preloadSpec;
        preloadSpec = preloadspecPlace.build();
        Action End;
        End = backUp.build();

        waitForStart();
        Thread slideThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                depositSubsystem.updateSlide();
                if(depositSubsystem.liftMotor1.getCurrentPosition()>2700 && depositSubsystem.liftMotor1.getCurrentPosition()<2800){
                    // Close claw
                    depositSubsystem.armPlace();
                    depositSubsystem.tiltPlacespec();
                }
                else if(depositSubsystem.liftMotor1.getCurrentPosition()> 2950){
                    depositSubsystem.tiltPlaceSpec();
                }
                else if(depositSubsystem.liftMotor1.getCurrentPosition()> 2600 && depositSubsystem.liftMotor1.getCurrentPosition()<2650){
                    depositSubsystem.armCollect();
                    depositSubsystem.tiltPlacec();

                }
            }
        });
        slideThread.start();
        depositSubsystem.setTargetSlide(3100);
        depositSubsystem.closeClaw(); // Close claw
        depositSubsystem.armPlace();
        depositSubsystem.tiltPlaceSpec();
        ElapsedTime closeClaw = new ElapsedTime();
        ElapsedTime collect = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("lift pos: ", depositSubsystem.getLiftPoz());
            telemetry.update();
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
                    stage = Stage.collect1M;
                    break;
                case collect1M:
                    Actions.runBlocking(
                            new SequentialAction(
                                    Collect1

                            )



                    );
                    depositSubsystem.setTargetSlide(0);
                    depositSubsystem.armPlace();
                    if(depositSubsystem.getLiftPoz()< 400) {
                        collectionSubsystem.collect();
                        collectionSubsystem.CRExtend();
                        collectionSubsystem.tiltRetract();

                        collect.reset();
                        stage = Stage.collect1S;
                    }
                    break;
                case collect1S:
                    if(collect.seconds()> 1.5&& collect.seconds()<2.5){
                        collectionSubsystem.CRRetract();
                        collectionSubsystem.stopCollection();
                        collectionSubsystem.tiltCollect();

                    }
                    if (collect.seconds()>3&& collect.seconds()<3.5){
                        collectionSubsystem.reverseCollection();
                        depositSubsystem.armCollect();
                        depositSubsystem.tiltPlacec();
                        depositSubsystem.openClaw();
                    }

                    if(collect.seconds()>4){
                        depositSubsystem.closeClaw();

                        stage = Stage.place1M;
                    }
                    break;
                case place1M:
                    depositSubsystem.setTargetSlide(3100);
                    depositSubsystem.armPlace();

                    Actions.runBlocking(
                            new SequentialAction(
                                    Place1

                            )


                    );
                    closeClaw.reset();
                    stage =Stage.place1S;
                    break;
                case place1S:
                    if(depositSubsystem.getLiftPoz()> 3050) {

                        depositSubsystem.openClaw();
                        stage = Stage.collect2M;
                    }


                    break;


                case collect2M:
                    Actions.runBlocking(
                            new SequentialAction(
                                    Collect2

                            )


                    );
                    depositSubsystem.setTargetSlide(0);
                    depositSubsystem.armPlace();
                    if(depositSubsystem.getLiftPoz()< 400) {
                        collectionSubsystem.collect();
                        collectionSubsystem.CRExtend();
                        collectionSubsystem.tiltRetract();

                        collect.reset();
                        stage = Stage.collect2S;
                    }
                    break;
                case collect2S:
                    if(collect.seconds()> 1.5&& collect.seconds()<2.5){
                        collectionSubsystem.CRRetract();
                        collectionSubsystem.stopCollection();
                        collectionSubsystem.tiltCollect();

                    }
                    if (collect.seconds()>3&& collect.seconds()<3.5){
                        collectionSubsystem.reverseCollection();
                        depositSubsystem.armCollect();
                        depositSubsystem.tiltPlacec();
                        depositSubsystem.openClaw();
                    }

                    if(collect.seconds()>4){
                        depositSubsystem.closeClaw();

                        stage = Stage.place2M;
                    }
                    break;
                case place2M:
                    depositSubsystem.setTargetSlide(3100);
                    depositSubsystem.armPlace();
                    Actions.runBlocking(
                            new SequentialAction(
                                    Place2

                            )


                    );
                    closeClaw.reset();
                    stage =Stage.place2S;
                    break;
                case place2S:
                    if(depositSubsystem.getLiftPoz()> 3050) {

                        depositSubsystem.openClaw();
                        stage = Stage.idle;
                    }
                    break;
                case collect3M:
                    collectionSubsystem.tiltRetract();
                    collectionSubsystem.collect();
                    depositSubsystem.setTargetSlide(0);
                    Actions.runBlocking(
                            new SequentialAction(
                                    Collect3

                            )


                    );

                    collect.reset();
                    stage =Stage.collect3S;
                    break;
                case collect3S:
                    if(collect.seconds()> .75&& collect.seconds()<1){
                        collectionSubsystem.stopCollection();
                        collectionSubsystem.tiltCollect();

                    }
                    if (collect.seconds()>1&& collect.seconds()<1.25){
                        collectionSubsystem.reverseCollection();
                        depositSubsystem.armCollect();
                        depositSubsystem.tiltPlacec();
                        depositSubsystem.openClaw();
                    }
                    if (collect.seconds()>1.75) {
                        collectionSubsystem.stopCollection();

                    }
                    if(collect.seconds()>2){
                        depositSubsystem.closeClaw();

                        stage = Stage.idle;
                    }
                    break;
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

