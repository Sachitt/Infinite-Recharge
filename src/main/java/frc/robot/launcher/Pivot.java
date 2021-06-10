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
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANPIDController;


public class Pivot {
    private CANSparkMax sparkA;
    private DigitalInput lowerLimit;

    private final double pkP = 0.01;
    private final double pkI = 0;
    private final double pkD = 0.2;
    private final double pkF = 0;


    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalInput(Constants.PIVOT_LIMIT_PORT);
        initShuffleBoard();
        
        sparkA.getPIDController().setP(pkP);
        sparkA.getPIDController().setI(pkI);
        sparkA.getPIDController().setD(pkD);
        sparkA.getPIDController().setFF(pkF);
    }

    //shuffleboard methods directly from Master Code
    public void initShuffleBoard(){
        SmartDashboard.putNumber("Pivot Rev", 0);
    }

    public void workShuffleBoard(){
        SmartDashboard.putNumber("Pivot Rev", rev());
    }

    public void reset() {
        sparkA.set(0);
        sparkA.getEncoder().setPosition(0);
    }


    public double rev() {
        return -sparkA.getEncoder().getPosition();
    }


    public void run() {
       
        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > Constants.TRIGGER_THRESHOLD) {
            teleRun();
        } else if (Robot.operatorController.getXButton()) {
            reset();
        } else {
            // dpad
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    PIDRun();
                    break;
                case 90:
                    PIDRun2();;
                    break;
                case 180:
                    // Down
                    break;
                case 270:
                    // Left
                    break;
                default:
                    stop();
            }
        }
        workShuffleBoard();
    }

    public void stop() {
        sparkA.set(0);
    }

    public void teleRun() {



        //limit switch and conditionals are from Master Code 
        if(!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft)> 0){

          reset();
          return;

        }
        if (rev() < Constants.PIVOT_ZERO_THRESHOLD && Robot.operatorController.getY(Hand.kLeft) > 0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_ZERO_SPEED);
            return;
        }


        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void PIDRun(){

      sparkA.getPIDController().setReference(-15, ControlType.kPosition);

    }

    public void PIDRun2(){

        sparkA.getPIDController().setReference(-20, ControlType.kPosition);
    };

}