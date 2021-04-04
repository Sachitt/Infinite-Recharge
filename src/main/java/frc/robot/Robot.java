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
  private static String pathName;


  private static int i = 0;
  public String fileName = "/tankvals.json";

  private static List<String> tankVals;

  public static List<String> ARED;
  public static List<String> ABLUE;
  public static List<String> BRED;
  public static List<String> BBLUE;

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

    

    pathName = drive.evaluatePath();
    System.out.println(pathName);

    // pathName = "BBLUE";
    
      if(pathName == "ARED") {
        tankVals = drive.buildARED();
      } else if (pathName == "ABLUE") {
        tankVals = drive.buildABLUE();
      } else if (pathName == "BRED") {
        tankVals = drive.buildBRED();
      } else if (pathName == "BBLUE") {
        tankVals = drive.buildBBLUE();
      }


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
<<<<<<< HEAD


      
      

    }
=======
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
    
>>>>>>> 2a8d4f38fa47d89597fc12bf6dc06f3ed33a5c19
    
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

  //}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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
