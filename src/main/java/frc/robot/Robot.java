/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.drivetrain.*;
import frc.robot.launcher.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static XboxController driverController;
  public static XboxController operatorController;

  //public static Climb climb;
  public static DriveTrain drivetrain;
  public static Intake intake;
  public static Pivot pivot;
  public static Shooter shooter;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driverController = new XboxController(Constants.DRIVER_PORT);
    operatorController = new XboxController(Constants.OPERATOR_PORT);

    drivetrain = new DriveTrain();
    intake = new Intake();
    pivot = new Pivot();
    shooter = new Shooter();
  }
  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
  }



  @Override
  public void autonomousPeriodic() {
  
  }

 
  @Override
  public void teleopPeriodic() {
    drivetrain.run();
    pivot.run();
    shooter.run();
    intake.run();
  }

  @Override
  public void testPeriodic() {
  }

}