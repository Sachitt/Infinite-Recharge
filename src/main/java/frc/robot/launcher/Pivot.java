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

public class Pivot {
    public static CANSparkMax sparkA;
    private DigitalInput lowerLimit;

    private final double pkP = 0.5;
    private final double pkI = 0;
    private final double pkD = 4;
    private final double pkF = 0;

    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalInput(Constants.PIVOT_LIMIT_PORT);

        sparkA.getPIDController().setP(pkP);
        sparkA.getPIDController().setI(pkI);
        sparkA.getPIDController().setD(pkD);
        sparkA.getPIDController().setFF(pkF);

        sparkA.setInverted(true);
    }

    public void SetShuffleboard() {
        SmartDashboard.putNumber("Pivot Value", 0);
    }

    public void reset() {
        sparkA.set(0);
        sparkA.getEncoder().setPosition(0);
    }

    public double rev() {
        return sparkA.getEncoder().getPosition();
    }

    public void run() {

        System.out.println(rev() + ", " + lowerLimit.get());
        // System.out.print;

        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > 0.2) {
            Run();
        } else {
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    PIDRun(25);
                    break;
                case 90:
                    PIDRun(30);
                    break;
                case 180:
                    // Down
                    PIDRun(20);
                    break;
                case 270:
                    // Left
                    PIDRun(15);
                    break;
                default:
                    stop();
            }
        }
    }

    public void stop() {
        sparkA.set(0);
    }

    public void Run() {

        if (!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft) > 0) {
            reset();
            return;
        }

        if (rev() < 10.0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * 0.1);
        }

        if (rev() < 5.0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * 0.05);
        }

        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void PIDRun(double rotations) {

        if (rev() < 10.0) {
            sparkA.set(0.05);
            return;
        }

        if (rev() < 5.0) {
            sparkA.set(0);
            return;
        }
        sparkA.getPIDController().setReference(rotations, ControlType.kPosition);
    }
}