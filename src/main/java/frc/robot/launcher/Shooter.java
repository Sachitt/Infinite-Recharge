package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;


public class Shooter {
    private CANSparkMax sparkA, sparkB; 
    private DoubleSolenoid shooterSolenoid; 
    private double Time; 

    public Shooter() {
        sparkA = new CANSparkMax(Constants.SHOOTER_PORTS[0], MotorType.kBrushless);
        sparkB = new CANSparkMax(Constants.SHOOTER_PORTS[1], MotorType.kBrushless);

        ArrayList<CANSparkMax> shooterSparkMax = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
            }
        };

        for (CANSparkMax spark : shooterSparkMax) {
            spark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setIdleMode(IdleMode.kCoast); 
        }

        shooterSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SHOOTER_PORT[0], Constants.PNEUMATIC_SHOOTER_PORT[1]);
        Time = 0;

        sparkA.getPIDController().setP(Constants.SHOOTER_KP);
        sparkA.getPIDController().setI(Constants.SHOOTER_KI);
        sparkA.getPIDController().setD(Constants.SHOOTER_KD);
        sparkA.getPIDController().setFF(Constants.SHOOTER_KF);
        sparkB.follow(sparkA, true);
    }

    public double getRPM() {
        return sparkA.getEncoder().getVelocity();
    }


    public void run() {
        if (Robot.operatorController.getTriggerAxis(Hand.kRight) >= Constants.TRIGGER_THRESHOLD) {
            shoot();
        } else {
            stop();
        }

        

    }


    public void shoot() {
        if (Time == 0) {
            Time = System.currentTimeMillis();
        }

        spinUp();

        if (System.currentTimeMillis() - Time > Constants.SHOOTER_SHOOT_TIME) {
            open();
        }
    }


    public void open() {
        shooterSolenoid.set(Value.kReverse);
    }

    public void lock() {
        shooterSolenoid.set(Value.kForward);
    }

    public void spinUp() {
        sparkA.getPIDController().setReference(Constants.SHOOTER_TARGET_RPM, ControlType.kVelocity);
    }

    public void spinDown() {
        sparkA.set(0);
    }
    public void stop() {
        lock();
        spinDown();
        Time = 0; 
    }

    
}