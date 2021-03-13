package com.team2568.frc2020.fsm.auto;

import com.team2568.frc2020.Constants;
import com.team2568.frc2020.Registers;
import com.team2568.frc2020.fsm.FSM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveTrain extends FSM {
    private static DriveTrain mInstance;

    private double tx;
    private double driveZ;

    private PIDController mAlignController, mVelocityController;

    private double mStart;

    private Trajectory mTrajectory;
    private double driveLV, driveRV;

    private RamseteController mRamController;
    private SimpleMotorFeedforward mVelocityFeedForward;

    public enum DriveAutoMode {
        kOff, kTarget, kTrajectory;
    }

    public static DriveTrain getInstance() {
        if (mInstance == null) {
            mInstance = new DriveTrain();
        }
        return mInstance;
    }

    private DriveTrain() {
        mAlignController = new PIDController(Constants.kDriveAlignkP, Constants.kDriveAlignkI, Constants.kDriveAlignkD);

        mVelocityController = new PIDController(Constants.kDriveVelocitykP, Constants.kDriveVelocitykI,
                Constants.kDriveVelocitykD);
        mVelocityFeedForward = new SimpleMotorFeedforward(Constants.kVolt, Constants.kVoltSecondPerMeter,
                Constants.kVoltSecondSquaredPerMeter);
        mRamController = new RamseteController(Constants.kDrivekB, Constants.kDrivekZeta);
    };

    public void compute() {
        update();

        switch (Registers.kDriveAutoMode.get()) {
        case kOff:
            driveZ = 0;
            driveLV = 0;
            driveRV = 0;

            // Do not reset controller while in operations
            mAlignController.reset();
            mStart = 0;
            break;
        case kTarget:
            driveLV = 0;
            driveRV = 0;
            driveZ = getDriveZ();
            break;
        case kTrajectory:
            mTrajectory = Registers.kDriveAutoTrajectory.get();
            driveZ = 0;

            if (mStart == 0) {
                mStart = Timer.getFPGATimestamp();
                Constants.kDriveHelper.reset(mTrajectory.getInitialPose());
            }

            DifferentialDriveWheelSpeeds wheelSpeeds = getSpeed();
            driveLV = getVoltage(Constants.kDriveHelper.encoderToMeter(Registers.kDriveLeftVelocity.get()) / 60,
                    wheelSpeeds.leftMetersPerSecond);
            driveRV = getVoltage(Constants.kDriveHelper.encoderToMeter(Registers.kDriveRightVelocity.get()) / 60,
                    wheelSpeeds.rightMetersPerSecond);
            break;
        }
        Registers.kDriveAutoDriveZ.set(driveZ);
        Registers.kDriveAutoLV.set(driveLV);
        Registers.kDriveAutoRV.set(driveRV);
    }

    private double getDriveZ() {
        return MathUtil.clamp(mAlignController.calculate(tx, 0), -Constants.kDriveMaxRotationSpeed,
                Constants.kDriveMaxRotationSpeed);
    }

    private DifferentialDriveWheelSpeeds getSpeed() {
        if (Timer.getFPGATimestamp() - mStart > mTrajectory.getTotalTimeSeconds()) {
            return new DifferentialDriveWheelSpeeds();
        }

        ChassisSpeeds chassisSpeed = mRamController.calculate(Registers.kDrivePose2d.get(),
                mTrajectory.sample(Timer.getFPGATimestamp() - mStart));
        return Constants.kDriveKinematics.toWheelSpeeds(chassisSpeed);
    }

    /**
     * Calcuates a voltage value with both PID and Feedforward components. The final
     * output is clamped by the maximum voltage
     * 
     * @param actualSpeed
     * @param referenceSpeed
     * @return
     */
    private double getVoltage(Double actualSpeed, Double referenceSpeed) {
        return MathUtil.clamp(
                mVelocityController.calculate(referenceSpeed, actualSpeed)
                        + mVelocityFeedForward.calculate(referenceSpeed),
                -Constants.kMaxVoltage, Constants.kMaxVoltage);
    }

    public void writeDashboard() {
        SmartDashboard.putString("DriveAutoMode", Registers.kDriveAutoMode.get().toString());
    }

    private void update() {
        Constants.kFrontLL.setPipeline(Constants.kUpperPort);

        tx = Constants.kFrontLL.getTx();
    }
}
