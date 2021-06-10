package frc.robot.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Robot;



public class DriveTrain{
    
    private CANSparkMax lMotorA, lmotorB,lmotorC, rmotorA, rmotorB, rmotorC;
    private final int RB_PORT = 6, LB_PORT = 3, LF_PORT = 4, RF_PORT = 9;
    private boolean switch1;
    DifferentialDrive testDrive;
    SpeedControllerGroup controllerL, controllerR;
   


    public DriveTrain() {
    lMotorA = new CANSparkMax(LB_PORT, MotorType.kBrushless);
    rmotorA = new CANSparkMax(RB_PORT, MotorType.kBrushless);
    lmotorB = new CANSparkMax(LF_PORT, MotorType.kBrushless);
    rmotorB = new CANSparkMax(RF_PORT, MotorType.kBrushless);
    lmotorC = new CANSparkMax(5, MotorType.kBrushless);
    rmotorC = new CANSparkMax(10, MotorType.kBrushless);

    controllerL = new SpeedControllerGroup(lMotorA, lmotorB, lmotorC);
    controllerR = new SpeedControllerGroup(rmotorA, rmotorB, rmotorC);

    

    testDrive = new DifferentialDrive(controllerL, controllerR);
    
    this.switch1 = false;  

}

public void TankDrive()
{

if (Robot.driverController.getAButtonPressed()) {
        switch1 = !switch1;
}
  //constants
   int R = 1;
   int L = 1;
   double axisR = Robot.driverController.getY(Hand.kRight);
   double axisL = Robot.driverController.getY(Hand.kLeft);

  if (axisR <0){
      R = 1;
  }
  else if (axisR > 0){
      R = -1;
  }

  if (axisL < 0){
    L = 1;
}
  else if (axisL > 0){
    L = -1;
}
double power = 0.8;



double leftD = L *power * Math.pow(Math.abs(axisL), 1.5);
double rightD = R * power * Math.pow(Math.abs(axisR), 1.5);



if (switch1) {
    testDrive.tankDrive(-rightD, -leftD);
} else {
    testDrive.tankDrive(leftD, rightD);
}

}

public void run()
{
    TankDrive();
}
 
        
    












}