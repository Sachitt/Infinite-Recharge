package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Class that handles the intake mechanism and the tube intake
 * 
 * <p>
 * Note: intake pneumatics should default to the upward position
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
public class Intake {
    private CANSparkMax sparkA, sparkB, sparkC, sparkD;
    private DoubleSolenoid intakeSolenoid;

    private boolean intake;

    public Intake() {
        // Intialize motors
        sparkA = new CANSparkMax(Constants.kIntakeLMotor, MotorType.kBrushless);
        sparkB = new CANSparkMax(Constants.kIntakeRMotor, MotorType.kBrushless);
        sparkC = new CANSparkMax(Constants.kTubeLMotor, MotorType.kBrushless);
        sparkD = new CANSparkMax(Constants.kTubeRMotor, MotorType.kBrushless);

        @SuppressWarnings("serial")
        ArrayList<CANSparkMax> shooterSparkMax = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
                add(sparkC);
                add(sparkD);
            }
        };

        for (CANSparkMax spark : shooterSparkMax) {
            spark.setSmartCurrentLimit(35);
            spark.setSecondaryCurrentLimit(35);
            spark.setIdleMode(IdleMode.kBrake);
        }

        intakeSolenoid = new DoubleSolenoid(Constants.kIntakeF, Constants.kIntakeR);

        intake = false;
    }

    public void run() {
        if (Constants.operatorController.getAButtonPressed()) {
            intake = !intake;
        }

        if (intake) {
            intakeDown();
        } else {
            intakeUp();
        }

        if (Constants.operatorController.getBumper(Hand.kLeft)) {
            tubeIntake();
        } else if (Constants.operatorController.getTriggerAxis(Hand.kLeft) >= Constants.kTriggerThreshold
                || Constants.operatorController.getTriggerAxis(Hand.kRight) >= Constants.kTriggerThreshold) {
            tubeShoot();
        } else if (Constants.operatorController.getBButton()) {
            tubeReverse();
        } else if (Constants.operatorController.getStartButtonPressed()) {
            intakeDown();
            intake();
        } else {
            tubeOff();
        }

        if (Constants.operatorController.getTriggerAxis(Hand.kLeft) >= Constants.kTriggerThreshold) {
            intake();
        } else if (Constants.operatorController.getBumper(Hand.kRight)) {
            intakeReverse();
        } else {
            intakeOff();
        }

    }

    public void intakeDown() {
        intakeSolenoid.set(Value.kForward);
    }

    public void intakeUp() {
        intakeSolenoid.set(Value.kReverse);
    }

    public void tubeIntake() {
        sparkC.set(Constants.kTubeIntakeSpeed);
        sparkD.set(-Constants.kTubeIntakeSpeed);
    }

    public void tubeShoot() {
        sparkC.set(Constants.kTubeShootSpeed);
        sparkD.set(-Constants.kTubeShootSpeed);
    }

    public void tubeReverse() {
        sparkC.set(-Constants.kTubeIntakeSpeed);
        sparkD.set(Constants.kTubeIntakeSpeed);
    }

    public void tubeOff() {
        sparkC.set(0);
        sparkD.set(0);
    }

    public void intake() {
        sparkA.set(Constants.kIntakeSpeed);
        sparkB.set(-Constants.kIntakeSpeed);
    }

    public void intakeReverse() {
        sparkA.set(-Constants.kIntakeSpeed);
        sparkB.set(Constants.kIntakeSpeed);
    }

    public void intakeOff() {
        sparkA.set(0);
        sparkB.set(0);
    }
}