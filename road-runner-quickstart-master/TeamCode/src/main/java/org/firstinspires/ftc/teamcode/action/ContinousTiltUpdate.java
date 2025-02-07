package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsytem.DepositSubsystem;

public abstract class ContinousTiltUpdate implements Action {
    private DepositSubsystem depositSubsystem;
    private boolean isFinished = false;

    public ContinousTiltUpdate(DepositSubsystem depositSubsystem) {
        this.depositSubsystem = depositSubsystem;
    }


    public boolean run(double time) {
        depositSubsystem.updateTilt(); // Continuously update tilt
        return isFinished; // Always return false to keep running
    }

    // Call this when you want to stop the tilt control
    public void stop() {
        isFinished = true;
    }
}
