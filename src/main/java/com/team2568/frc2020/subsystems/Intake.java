package com.team2568.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.team2568.frc2020.Constants;
import com.team2568.frc2020.Registers;
import com.team2568.lib.drivers.SparkMaxFactory;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * Two motors that work inverted spin a rod of wheels that feed the balls into
 * the tube. The rod is attached to pneumatic pistons which lower or raise the
 * rod to stay within the bounds of the robot.
 * </p>
 * 
 * <p>
 * IntakeValue: kForward, kReverse, kOff
 * </p>
 * 
 * <p>
 * IntakeDown: True/False
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
public class Intake extends Subsystem {
    private static Intake mInstance;

    private CANSparkMax lMotor, rMotor;
    private DoubleSolenoid solenoid;

    public static enum IntakeValue {
        kForward, kOff, kReverse;
    }

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    private Intake() {
        if (Registers.kReal.get()) {
            lMotor = SparkMaxFactory.getDefault(Constants.kIntakeLMotor);
            rMotor = SparkMaxFactory.getDefault(Constants.kIntakeRMotor);
        }

        solenoid = new DoubleSolenoid(Constants.kIntakeF, Constants.kIntakeR);
    }

    public void setOutputs() {
        if (Registers.kIntakeDown.get()) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }

        if (Registers.kReal.get()) {
            switch (Registers.kIntakeValue.get()) {
            case kOff:
                lMotor.set(0);
                rMotor.set(0);
                break;
            case kForward:
                lMotor.set(Constants.kIntakeSpeed);
                rMotor.set(-Constants.kIntakeSpeed);
                break;
            case kReverse:
                lMotor.set(-Constants.kIntakeSpeed);
                rMotor.set(Constants.kIntakeSpeed);
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
            SmartDashboard.putBoolean("IntakeDown", Registers.kIntakeDown.get());
            SmartDashboard.putString("IntakeValue", Registers.kIntakeValue.get().toString());
        }
    }
}
