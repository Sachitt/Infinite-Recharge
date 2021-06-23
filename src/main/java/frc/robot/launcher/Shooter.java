package frc.robot.launcher;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter {

    CANSparkMax spark1, spark2;
    DoubleSolenoid shooterLock;
    double Time;
    public static boolean pneumaticCheckS = false;

    public Shooter() {

        spark1 = new CANSparkMax(1, MotorType.kBrushless);
        spark2 = new CANSparkMax(2, MotorType.kBrushless);

        spark1.setSmartCurrentLimit(45);
        spark1.setSecondaryCurrentLimit(45);
        spark1.setIdleMode(IdleMode.kCoast);

        spark2.setSmartCurrentLimit(45);
        spark2.setSecondaryCurrentLimit(45);
        spark2.setIdleMode(IdleMode.kCoast);

        shooterLock = new DoubleSolenoid(1, 0);

        Time = 0;

        spark1.getPIDController().setP(Constants.SHOOTER_KP);
        spark1.getPIDController().setP(Constants.SHOOTER_KI);
        spark1.getPIDController().setP(Constants.SHOOTER_KD);
        spark1.getPIDController().setP(Constants.SHOOTER_KF);
        spark2.follow(spark1, true);

    }

    public double RPM() {
        // return spark1.getVelocity
        return spark1.getEncoder().getVelocity();
    }

    public void run() {
        if (Robot.operatorController.getTriggerAxis(Hand.kRight) >= 0.2) {

            shoot();
        } else {
            stop();

        }

        runPneumaticToggle();

    }

    public void shoot() {
        if (Time == 0) {

            Time = System.currentTimeMillis();

        }
        spinUp();

        if (System.currentTimeMillis() - Time > 720/* in github repo constant */)
            openSolenoid();
        // if(RPM() > 6000) openSolenoid();
    }

    /*
     * The Toggle below uses a static boolean called pnuematic check to check if the
     * pneumatics are oppened or closed. If its open, the right trigger will run
     * both the wheels and the belt. Used the boolean in the Intake class to check
     * if it is open or not.
     */

    public void runPneumaticToggle() {

        if (Robot.operatorController.getXButtonPressed()) {
            if (pneumaticCheckS == false) {
                openSolenoid();
                pneumaticCheckS = true;
            } else if (pneumaticCheckS == true) {
                lockSolenoid();
                pneumaticCheckS = false;
            }
        }
    }

    public void openSolenoid() {
        shooterLock.set(Value.kReverse);
    }

    public void lockSolenoid() {
        shooterLock.set(Value.kForward);
    }

    public void stop() {
        lockSolenoid();
        spinDown();
        Time = 0;

    }

    public void spinUp() {
        /*
         * if (RPM() - 4500 > 50) { spark1.set(0.5); spark2.set(-0.5); } else {
         * spark1.set(0.8); spark2.set(-0.8); }
         */
        double TargetRPM;
        boolean speedToggle = true;

        if (Robot.operatorController.getStartButtonPressed() && speedToggle) {
            speedToggle = !speedToggle;
        }
        if (speedToggle) {
            TargetRPM = 4500;
        } else {
            TargetRPM = 3500;
        }

        spark1.getPIDController().setReference(TargetRPM, ControlType.kVelocity);

    }

    public void spinDown() {
        spark1.set(0);
        spark2.set(0);
    }

}
