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

  public static Climb climb;
  public static DriveTrain drivetrain;
  public static Intake intake;
  public static Pivot pivot;
  public static Shooter shooter;

  private boolean pivotAligned;
  private double startTime, moveTime;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driverController = new XboxController(Constants.DRIVER_PORT);
    operatorController = new XboxController(Constants.OPERATOR_PORT);

    climb = new Climb(operatorController);
    drivetrain = new DriveTrain();
    intake = new Intake();
    pivot = new Pivot();
    shooter = new Shooter();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    pivotAligned = false;
    startTime = 0;
    moveTime = 0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    pivotAligned = pivot.atLine();
    if (!pivotAligned) {
      pivot.setLine();
      return;
    }
    
    pivot.stop();

    if (startTime == 0) {
      startTime = System.currentTimeMillis();
    }

    if (System.currentTimeMillis() - startTime < Constants.AUTO_SHOOT_TIME) {
      shooter.shoot();
      intake.tubeShoot();
      return;
    }

    shooter.stop();
    intake.tubeOff();

    if (moveTime == 0) {
      moveTime = System.currentTimeMillis();
    }

    if (System.currentTimeMillis() - moveTime < Constants.AUTO_MOVE_TIME) {
      drivetrain.moveForward();
      return;
    }

    drivetrain.stop();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    climb.run();
    drivetrain.run();
    pivot.run();
    shooter.run();
    intake.run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
