package frc.robot.launcher;

import java.util.ArrayList;
import java.util.Collections;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Class that handles the pivot/screw mechanism that rotates the launcher
 * mechanism
 * 
 * <p>
 * Note: (- = upwards, + = downwards)
 * 
 * @author Ryan Chaiyakul
 */
public class Pivot {
    private CANSparkMax sparkA; // Spark to control pivot location with screw mechanism
    private Alignment alignment; // Alignment object
    private DigitalInput lowerLimit; // lowerLimit is active low

    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalInput(Constants.PIVOT_LIMIT_PORT);
        alignment = new Alignment();
        initShuffleBoard();

        sparkA.getPIDController().setP(Constants.PIVOT_KP);
        sparkA.getPIDController().setI(Constants.PIVOT_KI);
        sparkA.getPIDController().setD(Constants.PIVOT_KD);
        sparkA.getPIDController().setFF(Constants.PIVOT_KF);

        // Gravity assists downward motion, so auto speed downwards is less
        sparkA.getPIDController().setOutputRange(-Constants.PIVOT_AUTO_SPEED - 0.05, Constants.PIVOT_AUTO_SPEED);
    }

    public void initShuffleBoard() {
        SmartDashboard.putNumber("Pivot Rev", 0);
    }

    public void workShuffleBoard() {
        SmartDashboard.putNumber("Pivot Rev", getRevolution());
    }

    public void callibrate() {
        sparkA.set(0);
        sparkA.getEncoder().setPosition(0);
    }

    /**
     * Converted revolutions to a positive number
     * 
     * @return
     */
    public double getRevolution() {
        return -sparkA.getEncoder().getPosition();
    }

    public boolean atRev(double angle) {
        return Math.abs(angle - getRevolution()) < Constants.PIVOT_THRESHOLD;
    }

    public void run() {
        // System.out.println(getRevolution());
       
        workShuffleBoard();
        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > Constants.TRIGGER_THRESHOLD) {
            System.out.println("Current angle: " + getRevolution());
            teleopRun();
        } else if (Robot.operatorController.getStickButton(Hand.kRight)) {
            align();
        } else if (Robot.operatorController.getXButton()) {
            callibrate();
        }  else if (Robot.operatorController.getStickButton(Hand.kLeft)) {
            reset();
            
        }   else {
            // DPad controls
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    //setAgainst();
                    setGreen();
                    break;
                case 90:
                    // Right
                    //setWheel();
                    setYellow();
                    break;
                case 180:
                    // Down
                    //setTrench();
                    setRed();
                    break;
                case 270:
                    // Left
                    //setLine();
                    setBlue();
                    break;
                default:
                    // Catches -1 or status when nothing is pressed on DPad
                    stop();
            }
        }
    }

    private double angleToRev(double angle) {
        return 1.98 * angle - 51.7;
    }

    private double revToAngle(double rev){
        return (rev+51.7)/1.98;
    }

    public void stop() {
        sparkA.set(0);
    }

    public void reset() {
        setRevolution(Constants.PIVOT_ZERO_THRESHOLD);
    }

    public void teleopRun() {
        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft) > 0) {
            callibrate();
            return;
        }

        if (getRevolution() < Constants.PIVOT_ZERO_THRESHOLD && Robot.operatorController.getY(Hand.kLeft) > 0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_ZERO_SPEED);
            return;
        }

        if (getRevolution() > Constants.PIVOT_MAX_REVOLUTION && Robot.operatorController.getY(Hand.kLeft) < 0) {
            return;

        }
        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void align() {
        double theta_val = alignment.get_theta();
        setRevolution((angleToRev(theta_val)));
        Constants.whichSpeed = 0;
        /*
        if(theta_val >= 20) {
            setRevolution((angleToRev(theta_val)));
        } else {
            //call other function
            Shooter.shooterTarget = Constants.SHOOTER_SLOW_TARGET_RPM;
            double second_theta_val = alignment.get_second_theta();
            setRevolution(angleToRev(second_theta_val));
        }
        */
    }

    public void setGreen(){
        setRevolution(Constants.GREEN_ZONE);
        Constants.whichSpeed = 2;
        System.out.println("Position: Green, Rev: " + getRevolution());
    }

    public void setYellow(){
        setRevolution(Constants.YELLOW_ZONE);
        Constants.whichSpeed = 0;
        System.out.println("Position: Yellow, Rev: " + getRevolution());
    }

    public void setBlue(){
        setRevolution(Constants.BLUE_ZONE);
        Constants.whichSpeed = 0;
        System.out.println("Position: Blue, Rev: " + getRevolution());
    }

    public void setRed(){
        setRevolution(Constants.RED_ZONE);
        Constants.whichSpeed = 1;
        System.out.println("Position: Red");
    }
//------------------------------------------------------
    public void setAgainst() {
        setRevolution(79);
    }

    public void setLine() {
        setRevolution(16.35);
    }

    public void setWheel() {
        setRevolution(33);
    }

    public void setTrench() {
        setRevolution(7.5);
    }

    public boolean atAlign() {
        return atRev(alignment.getAngle() * Constants.RADIANS_TO_REV);
    }

    public boolean atAgainst() {
        return atRev(79);
    }

    public boolean atLine() {
        return atRev(19.5);
    }

    public boolean atWheel() {
        return atRev(33);
    }

    public boolean atTrench() {
        return atRev(12.13);
    }

    public void setRevolution(double rev) {
        // System.out.println(getRevolution() + ", " + rev);
        
        double targetRev = rev;
       
        // We can prevent the issue of zeroing correctly by preventing the PID
        // Controller from setting a value below the zero threshold

        if (rev < Constants.PIVOT_ZERO_THRESHOLD) {
            targetRev = Constants.PIVOT_ZERO_THRESHOLD;
        } else if (rev > Constants.PIVOT_MAX_REVOLUTION) {
            targetRev = Constants.PIVOT_MAX_REVOLUTION;
        }

        


        sparkA.getPIDController().setReference(-targetRev, ControlType.kPosition);
    }


}