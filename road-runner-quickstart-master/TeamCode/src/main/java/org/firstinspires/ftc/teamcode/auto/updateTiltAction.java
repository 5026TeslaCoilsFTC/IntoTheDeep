//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.roadrunner.Action;
//import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;
//
//public abstract class updateTiltAction implements Action {
//    private  DepositSubsystem depositSubsystem;
//
//    public updateTiltAction(DepositSubsystem depositSubsystem) {
//        this.depositSubsystem = depositSubsystem;
//    }
//
//
//    public boolean run(double time) {
//        depositSubsystem.updateTilt(); // Continuously update the tilt motor power
//        return false; // Return false to keep running until interrupted
//    }
//}