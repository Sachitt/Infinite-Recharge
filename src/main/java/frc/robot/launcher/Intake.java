package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake {
    private CANSparkMax sparkB, sparkC, sparkD;
    private DoubleSolenoid Solenoid;

    private boolean intake;

    public Intake() {

        sparkB = new CANSparkMax(Constants.INTAKE_PORTS[1], MotorType.kBrushless);
        sparkC = new CANSparkMax(Constants.INTAKE_PORTS[2], MotorType.kBrushless);
        sparkD = new CANSparkMax(Constants.INTAKE_PORTS[3], MotorType.kBrushless);

        ArrayList<CANSparkMax> shooter = new ArrayList<CANSparkMax>() {
            {
                add(sparkB);
                add(sparkC);
                add(sparkD);
            }
        };

        for (CANSparkMax spark : shooter) {
            spark.setSmartCurrentLimit(35);
            spark.setSecondaryCurrentLimit(35);
            spark.setIdleMode(IdleMode.kBrake);
        }

        Solenoid = new DoubleSolenoid(Constants.PNEUMATIC_INTAKE_PORT[0], Constants.PNEUMATIC_INTAKE_PORT[1]);

        intake = false;
    }

    public void run() {
        if (Robot.operatorController.getAButtonPressed()) {
            intake = !intake;
        }

        if (intake) {
            intakeDown();
        } else {
            intakeUp();
        }

        if (Robot.operatorController.getTriggerAxis(Hand.kLeft) >= Constants.TRIGGER_THRESHOLD) {
            intake();
        } else if (Robot.operatorController.getBumper(Hand.kRight)) {
            Reverse();
        } else {
            Off();
        }
        if (Robot.operatorController.getBumper(Hand.kLeft)) {
            tubeIntake();
        } else if (Robot.operatorController.getTriggerAxis(Hand.kLeft) >= Constants.TRIGGER_THRESHOLD
                || Robot.operatorController.getTriggerAxis(Hand.kRight) >= Constants.TRIGGER_THRESHOLD) {
            tubeShoot();
        } else if (Robot.operatorController.getBButton()) {
            tubeReverse();
        } else {
            tubeOff();
        }

    }

    public void intakeDown() {
        Solenoid.set(Value.kForward);
    }

    public void intakeUp() {
        Solenoid.set(Value.kReverse);
    }

    public void tubeIntake() {
        sparkC.set(Constants.INTAKE_TUBE_SPEED);
        sparkD.set(-Constants.INTAKE_TUBE_SPEED);
    }

    public void tubeShoot() {
        sparkC.set(Constants.INTAKE_SHOOT_SPEED);
        sparkD.set(-Constants.INTAKE_SHOOT_SPEED);
    }

    public void tubeReverse() {
        sparkC.set(-Constants.INTAKE_TUBE_SPEED);
        sparkD.set(Constants.INTAKE_TUBE_SPEED);
    }

    public void tubeOff() {
        sparkC.set(0);
        sparkD.set(0);
    }

    public void intake() {
        sparkB.set(-Constants.INTAKE_INTAKE_SPEED);
    }

    public void Reverse() {
        sparkB.set(Constants.INTAKE_INTAKE_SPEED);
    }

    public void Off() {
        sparkB.set(0);
    }
}