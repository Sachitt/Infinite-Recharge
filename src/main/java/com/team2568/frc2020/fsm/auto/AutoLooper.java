package com.team2568.frc2020.fsm.auto;

import com.team2568.frc2020.ILooper;
import com.team2568.frc2020.Registers;

public class AutoLooper extends ILooper {
    private static AutoLooper mLooper;

    public static AutoLooper getInstance() {
        if (mLooper == null) {
            mLooper = new AutoLooper();
        }
        return mLooper;
    }

    private AutoLooper() {
        super("Auto");
        registerRunnables(DriveTrain.getInstance().getRunnable(), Pivot.getInstance().getRunnable());
        registerStoppableRegisters(Registers.kDriveAutoMode, Registers.kPivotAutoMode, Registers.kDriveAutoTrajectory, Registers.kDriveAutoTankList);
        registerUpdateRegisters(Registers.kDriveAutoDriveZ, Registers.kDriveAutoLV, Registers.kDriveAutoRV, Registers.kDriveAutoL, Registers.kDriveAutoR);
        registerUpdateRegisters(Registers.kPivotAutoTargetRev);
    }
}
