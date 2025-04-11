package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsytem.CollectionSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsytem.DriveSubsystem;
/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "4 Bucket PP", group = "Examples")
public class bucket3 extends OpMode {
    private DepositSubsystem depositSubsystem;
    private DriveSubsystem driveSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    ElapsedTime action = new ElapsedTime();
    public static boolean scored = false;
    public static boolean actionIsReset = false;
    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(17, 123, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(26, 117.25, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(26, 127.25, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(47, 113.5, Math.toRadians(90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
    private boolean drove = false;

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()&& depositSubsystem.getLiftPoz()>2900 && drove&& action.milliseconds()>2000) {
                    score(1);
                }
                if(!follower.isBusy() && !drove) {
                    follower.followPath(scorePreload);
                    action.reset();
                    drove = true;
                }

                if(scored == true) {
                    setPathState(1);
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                if(!follower.isBusy()){
                    collectionSubsystem.tiltServo1.setPosition(collectionSubsystem.TILT_DOWN_POSITION);
                    follower.followPath(grabPickup1,true);

                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX()>20 && scored) {
                    /* Score Preload */
                    depositSubsystem.setTargetSlide(-50);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && !collectionSubsystem.clawClosed&& depositSubsystem.getTargetSlide()<50){
                    action.reset();

                    collectionSubsystem.updateState(true);

                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()&& CollectionSubsystem.clawClosed&& action.seconds()> 1) {
                    /* Grab Sample */
                    scored = false;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    depositSubsystem.setTargetSlide( 3050);
                    setPathState(3);
                }
                break;
            case 3:
                if(!actionIsReset){
                    action.reset();
                    actionIsReset = true;
                }
                if (action.seconds()> 2&& !scored&& !follower.isBusy()){
                score(1);
            }
                CollectionSubsystem.clawClosed = false;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()&& scored) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);

                }
                if(scored && follower.getPose().getX() > 23){
                    depositSubsystem.setTargetSlide(-50);
                    collectionSubsystem.tiltServo1.setPosition(collectionSubsystem.TILT_DOWN_POSITION);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy() && !collectionSubsystem.clawClosed&& depositSubsystem.getLiftPoz()<50){
                    action.reset();
                    collectionSubsystem.updateState(true);

                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()&& CollectionSubsystem.clawClosed && action.seconds()>2) {
                    actionIsReset = false;
                    depositSubsystem.setTargetSlide(3050);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    scored = false;
                    setPathState(5);
                }
                break;
            case 5:
                if(!actionIsReset){
                    action.reset();
                    actionIsReset = true;
                }
                if(actionIsReset&& action.seconds()>2) {
                    score(1);
                }
                collectionSubsystem.clawClosed = false;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && scored) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);

                }
                if(scored && follower.getPose().getX()> 23){
                    collectionSubsystem.tiltServo1.setPosition(collectionSubsystem.TILT_DOWN_POSITION);
                    depositSubsystem.setTargetSlide(-50);
                    setPathState(6);
                }
                break;
            case 6:

                if(!follower.isBusy() && !collectionSubsystem.clawClosed&& depositSubsystem.getLiftPoz()<50){
                    action.reset();
                    collectionSubsystem.updateState(true);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()&& CollectionSubsystem.clawClosed && action.seconds()>2) {
                    /* Grab Sample */
                    scored = false;
                    actionIsReset = false;
                    depositSubsystem.setTargetSlide(3050);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!actionIsReset){
                    action.reset();
                    actionIsReset = true;
                }
                if(actionIsReset && action.seconds() >2) {
                    score(1);
                }
                collectionSubsystem.clawClosed = false;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()&& scored) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(8);
                }
                if(follower.getPose().getX() > 23 && scored){
                    setPathState(8);
                }
                break;
            case 8:
                depositSubsystem.setTargetSlide(-50);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    waitMilliseconds(100);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        if(depositSubsystem.liftMotor1.getCurrentPosition()>500 && depositSubsystem.liftMotor1.getCurrentPosition()<2500){
            // Close claw
                depositSubsystem.armPlace();
        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 2600){

                depositSubsystem.tiltPlaceSpec();


        }
        else if(depositSubsystem.liftMotor1.getCurrentPosition()> 250 && depositSubsystem.liftMotor1.getCurrentPosition()<450){
            depositSubsystem.armCollect();
            depositSubsystem.tiltPlacec();
        }
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        depositSubsystem.updateSlide();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        depositSubsystem = new DepositSubsystem(hardwareMap, telemetry);
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        collectionSubsystem = new CollectionSubsystem(hardwareMap, telemetry, depositSubsystem);
        collectionSubsystem.tiltServo1.setPosition(collectionSubsystem.TILT_UP_POSITION);
        collectionSubsystem.extensionServo1.setPosition(collectionSubsystem.MIN_EXTENSION);
        depositSubsystem.armMiddle();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        depositSubsystem.setTargetSlide(3050);
        depositSubsystem.closeClaw();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public void waitMilliseconds(long milliseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds()<milliseconds){
            follower.update();
        }
    }
    public void score(int path){
        depositSubsystem.setTargetSlide(3050);
        if(depositSubsystem.getLiftPoz()>= 2900){
            telemetry.addLine("Claw is supposed to open");
            telemetry.addData("Claw Pos: ", depositSubsystem.clawServo.getPosition());
            depositSubsystem.openClaw();
        }
        if(depositSubsystem.clawServo.getPosition()== .15){
            telemetry.addLine("Scoring is set to true ");
            scored = true;
        }
    }
}

