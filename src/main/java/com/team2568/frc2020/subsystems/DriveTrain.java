package com.team2568.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.team2568.frc2020.Constants;
import com.team2568.frc2020.Registers;
import com.team2568.lib.drivers.SparkMaxFactory;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * Six motors with three on each side of the robot chassis are linked as two
 * speed controller groups for each side. This robot uses DifferentialDrive
 * which allows two driving modes: TankDrive and ArcadeDrive.
 * </p>
 * 
 * <p>
 * DriveMode: kTank, kArcade, kOff
 * </p>
 * 
 * <p>
 * TankDrive
 * </p>
 * 
 * <p>
 * DriveL: [-1, 1]
 * </p>
 * 
 * <p>
 * DriveR: [-1, 1]
 * </p>
 * 
 * <p>
 * Arcade Drive
 * </p>
 * 
 * <p>
 * DriveF: [-1, 1]
 * </p>
 * 
 * <p>
 * DriveZ: [-1, 1]
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
public class DriveTrain extends Subsystem {
    private static DriveTrain mInstance;

    private double driveL, driveR, driveF, driveZ, driveLV, driveRV;

    private CANSparkMax laMotor, lbMotor, lcMotor, raMotor, rbMotor, rcMotor;
    private SpeedControllerGroup lGroup, rGroup;

    private DifferentialDrive mDrive;

    public static enum DriveMode {
        kTank, kArcade, kDifferential, kOff;
    }

    public static DriveTrain getInstance() {
        if (mInstance == null) {
            mInstance = new DriveTrain();
        }
        return mInstance;
    }

    private DriveTrain() {
        if (Registers.kReal.get()) {
            laMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainLAMotor);
            lbMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainLBMotor);
            lcMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainLCMotor);

            lGroup = new SpeedControllerGroup(laMotor, lbMotor, lcMotor);

            raMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainRAMotor);
            rbMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainRBMotor);
            rcMotor = SparkMaxFactory.getDefault(Constants.kDriveTrainRCMotor);

            rGroup = new SpeedControllerGroup(raMotor, rbMotor, rcMotor);

            Constants.kDriveHelper.registerEncoders(laMotor.getEncoder(), raMotor.getEncoder());

            mDrive = new DifferentialDrive(lGroup, rGroup);
        }
    }

    public void setOutputs() {
        driveL = applyLimit(Registers.kDriveL.get());
        driveR = applyLimit(Registers.kDriveR.get());
        driveF = applyLimit(Registers.kDriveF.get());
        driveZ = applyLimit(Registers.kDriveZ.get());

        driveLV = Registers.kDriveLV.get();
        driveRV = Registers.kDriveRV.get();

        if (Registers.kReal.get()) {
            switch (Registers.kDriveMode.get()) {
            case kTank:
                mDrive.tankDrive(driveL, driveR);
                break;
            case kArcade:
                mDrive.arcadeDrive(driveF, driveZ);
                break;
            case kDifferential:
                differentialDrive(driveLV, driveRV);
                break;
            case kOff:
                mDrive.stopMotor();
                break;
            }
        }
    }

    // Speeds in MKS (Meters per second)
    private void differentialDrive(double leftSpeed, double rightSpeed) {
        lGroup.setVoltage(leftSpeed);
        rGroup.setVoltage(rightSpeed);
        mDrive.feed();
    }

    private double getLeftPosition() {
        if (Registers.kReal.get()) {
            return laMotor.getEncoder().getPosition();
        }
        return 0;
    }

    private double getRightPosition() {
        if (Registers.kReal.get()) {
            return raMotor.getEncoder().getPosition();
        }
        return 0;
    }

    private double getLeftVelocity() {
        if (Registers.kReal.get()) {
            return laMotor.getEncoder().getVelocity();
        }
        return 0;
    }

    private double getRightVelocity() {
        if (Registers.kReal.get()) {
            return raMotor.getEncoder().getVelocity();
        }
        return 0;
    }

    public void writeStatus() {
        Registers.kDriveLeftPosition.set(getLeftPosition());
        Registers.kDriveRightPosition.set(getRightPosition());
        Registers.kDriveLeftVelocity.set(getLeftVelocity());
        Registers.kDriveRightVelocity.set(getRightVelocity());

        if (Registers.kReal.get()) {
            Constants.kDriveHelper.update();

            Registers.kDrivePose2d.set(Constants.kDriveHelper.getPose());
        } else {
            Registers.kDrivePose2d.set(new Pose2d());
        }

    }

    public void writeDashboard() {
        SmartDashboard.putNumber("DriveLeftVelocity", Constants.kDriveHelper.encoderToMeter(getLeftVelocity()) / 60);
        SmartDashboard.putNumber("DriveRightVelocity", Constants.kDriveHelper.encoderToMeter(getRightVelocity()) / 60);
    }

    public void outputTelemetry() {
        if (!Registers.kReal.get()) {
            SmartDashboard.putString("DriveMode", Registers.kDriveMode.get().toString());
            SmartDashboard.putNumber("DriveL", driveL);
            SmartDashboard.putNumber("DriveR", driveR);
            SmartDashboard.putNumber("DriveF", driveF);
            SmartDashboard.putNumber("DriveZ", driveZ);
            SmartDashboard.putNumber("DriveLV", driveLV);
            SmartDashboard.putNumber("DriveRV", driveRV);
        } else {
            SmartDashboard.putNumber("DriveLeftPosition", Constants.kDriveHelper.encoderToMeter(getLeftPosition()));
            SmartDashboard.putNumber("DriveRightPosition", Constants.kDriveHelper.encoderToMeter(getRightPosition()));
        }
    }
}
