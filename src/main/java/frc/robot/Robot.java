// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.FileNotFoundException;
import java.io.FileReader;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DriveTrain.TankValue;
import org.json.*;
import org.json.simple.parser.JSONParser;
import java.io.*;
import java.util.*;

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

  private List<TankValue> tankValue = new ArrayList<TankValue>();
  private List<String> tankValuesString = new ArrayList<String>();
  private int i;
  public String fileName = "/tankvals.json";

  public static List<String> tankValues = new ArrayList<String>();

  JSONObject jsonObject;
  ObjectMapper mapper = new ObjectMapper();
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    drive = new DriveTrain();
    intake = new Intake();
    
    
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!-1.1435399445543137E-4");
    tankValues.add("-3.982037371533506E-4!-0.0020720362410402153");
    tankValues.add("-0.0037968882170629813!-0.025124957255636696");
    tankValues.add("-0.07977408521548603!-0.5493292248547015");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.018752508720997554!-0.10386427421437142");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("-0.01322153270454638!-0.016813855400741644");
    tankValues.add("-0.01322153270454638!-0.016813855400741644");
    tankValues.add("-0.08359256306506152!-0.5228945471659053");
    tankValues.add("-0.31154757042386644!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.7!-0.7");
    tankValues.add("-0.09146654454281344!-0.08359256306506152");
    tankValues.add("-0.0013866259507418844!0.0");
    tankValues.add("1.1275092010998849E-4!0.005885137466266466");
    tankValues.add("1.1275092010998849E-4!0.020493238945333586");
    tankValues.add("0.009877484060721586!0.09418219706445977");
    tankValues.add("0.06438462126095093!0.17897498938778977");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.7");
    tankValues.add("0.7!0.3421506007632024");
    tankValues.add("0.7!0.0013671874999999997");
    tankValues.add("0.7!-0.31154757042386644");
    tankValues.add("0.7!-0.603928986471218");
    tankValues.add("0.7!-0.6511770002995723");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("0.7!-0.7");
    tankValues.add("-0.1354807632385433!-0.7");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    tankValues.add("0.0!0.0");
    
    



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
    for (int i = 0; i<tankValues.size(); i++) {
      System.out.println(tankValues.get(i));
    }
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
    i = 0;
    
    if (Constants.kBackLL.getTx() < 0) {
      if (Constants.kBackLL.getTy() < 0) {
        // System.out.println("br");
      } else {
        // System.out.println("bb");
      }
    } else {
      if (Constants.kBackLL.getTy() < 0) {
        // System.out.println("ar");
      } else {
        // System.out.println("ab");
      }
    }
    
    
    // FileReader reader;
    // try {
    //   reader = new FileReader("src/main/java/frc/robot/tankvals.json");
    // } catch (FileNotFoundException e) {
    //   e.printStackTrace();
    // }
    // JSONParser jsonParser = new JSONParser();
    // jsonObject = (JSONObject) jsonParser.parse(reader);
    // System.out.println(jsonObject);
    // tankValues = new ArrayList<TankValue>();
    // // tankValues = mapper.readValue(Paths.get("src/main/deploy/tankvals.json"), TankValue.class);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Run intake and tube
    intake.intakeDown();
    intake.tubeIntake();
    intake.intake();

    // Iterate over tankValues until all motor values are set. Then stop until
    // autonomous is disabled
    if (i < tankValue.size() - 1) {
      drive.tankDrive(tankValue.get(i).getL(), tankValue.get(i).getR());
    } else {
      drive.stop();
    }

    i++;
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
