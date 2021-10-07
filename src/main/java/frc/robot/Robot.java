// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.drivetrain.*;

import java.util.List;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.json.*;
import frc.robot.launcher.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private DriveTrain drive;
  private Intake intake;

  public static Climb climb;
  public static Pivot pivot;
  public static Shooter shooter;

  private boolean pivotAligned;
  private double startTime;

  private boolean initialFlag;

  private static String pathName;

  private static int i = 0;
  public static boolean ending = false;
  public String fileName = "/tankvals.json";

  private static List<String> tankVals;

  public static List<String> ARED;
  public static List<String> ABLUE;
  public static List<String> BRED;
  public static List<String> BBLUE;

  JSONObject jsonObject;
  ObjectMapper mapper = new ObjectMapper();

  SendableChooser<Integer> autoChoice;

  public static int autoPath;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    drive = new DriveTrain();
    intake = new Intake();
    climb = new Climb(Constants.operatorController);
    pivot = new Pivot();
    shooter = new Shooter();

    // autoChoice = new SendableChooser<Integer>();
    // autoChoice.addOption("threeTrench", 1);
    // autoChoice.addOption("fiveTrench", 2);

    // autoPath = autoChoice.getSelected().intValue();
    // if (autoPath == 1) {
    // tankVals = drive.threeTrench();
    // }
    // pathName = drive.evaluatePath();
    // if(pathName == "ARED") {
    // tankVals = drive.buildARED();
    // } else if (pathName == "ABLUE") {
    // tankVals = drive.buildABLUE();
    // } else if (pathName == "BRED") {
    // tankVals = drive.buildBRED();
    // } else if (pathName == "BBLUE") {
    // tankVals = drive.buildBBLUE();
    // }
    tankVals = drive.threeTrench();

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

    initialFlag = false;
  }

  // FileReader reader;
  // try {
  // reader = new FileReader("src/main/java/frc/robot/tankvals.json");
  // } catch (FileNotFoundException e) {
  // e.printStackTrace();
  // }
  // JSONParser jsonParser = new JSONParser();
  // jsonObject = (JSONObject) jsonParser.parse(reader);
  // System.out.println(jsonObject);
  // tankValues = new ArrayList<TankValue>();
  // // tankValues = mapper.readValue(Paths.get("src/main/deploy/tankvals.json"),
  // TankValue.class);

  // }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (!initialFlag) {
      initialTeleop();
    } else {
      // Run intake and tube
      intake.intakeDown();
      intake.tubeIntake();
      intake.intake();

      // Iterate over tankValues until all motor values are set. Then stop until
      // autonomous is disabled
      if (i < tankVals.size() - 1) {
        String[] templist = tankVals.get(i).split("!");
        drive.tankDrive(Double.parseDouble(templist[0]), Double.parseDouble(templist[1]));
      } else {
        drive.stop();
        ending = true;
      }

      if (ending == true) {

      }

      i++;
    }

  }

  private void initialTeleop() {
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

    initialFlag = true;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.run();
    intake.run();
    climb.run();
    pivot.run();
    shooter.run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
