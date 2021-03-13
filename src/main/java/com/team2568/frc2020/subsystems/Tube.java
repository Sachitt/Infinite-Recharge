package com.team2568.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.team2568.frc2020.Constants;
import com.team2568.frc2020.Registers;
import com.team2568.lib.drivers.SparkMaxFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * Two motors that work inverted to each other turn two elastic tubings that
 * compresses and pushes the balls into the shooting lock.
 * </p>
 * 
 * <p>
 * TubeValue: kOff, kIntake, kShoot, kReverse
 * </p>
 * 
 * <p>
 * KIntake is slower than kShoot
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
public class Tube extends Subsystem {
    private static Tube mInstance;

    // rMotor is a follower so it will not be set explicitly
    @SuppressWarnings("unused")
    private CANSparkMax lMotor, rMotor;

    public enum TubeValue {
        kOff, kIntake, kShoot, kReverse;
    }

    public static Tube getInstance() {
        if (mInstance == null) {
            mInstance = new Tube();
        }
        return mInstance;
    }

    private Tube() {
        if (Registers.kReal.get()) {
            lMotor = SparkMaxFactory.getDefault(Constants.kTubeLMotor);
            rMotor = SparkMaxFactory.getInvertedFollower(lMotor, Constants.kTubeRMotor);
        }
    }

    public void setOutputs() {
        if (Registers.kReal.get()) {
            switch (Registers.kTubeValue.get()) {
            case kOff:
                lMotor.set(0);
                break;
            case kIntake:
                lMotor.set(Constants.kTubeIntakeSpeed);
                break;
            case kShoot:
                lMotor.set(Constants.kTubeShootSpeed);
                break;
            case kReverse:
                lMotor.set(-Constants.kTubeIntakeSpeed);
                break;
            }
        }
    }

    public void writeStatus() {
    }

    public void writeDashboard() {
    }

    public void outputTelemetry() {
        if (!Registers.kReal.get()) {
            SmartDashboard.putString("TubeValue", Registers.kTubeValue.get().toString());
        }
    }
}
