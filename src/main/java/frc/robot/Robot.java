/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.team2568.frc2020.Registers;
import com.team2568.frc2020.fsm.auto.AutoLooper;
import com.team2568.frc2020.fsm.auto.DriveTrain.DriveAutoMode;
import com.team2568.frc2020.fsm.teleop.TeleopLooper;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.regex.Pattern;

import com.team2568.frc2020.Constants;

import com.team2568.frc2020.ILooper;
import com.team2568.frc2020.subsystems.SubsystemLooper;
import com.team2568.frc2020.subsystems.DriveTrain.DriveMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private ILooper teleopLooper, subsystemLooper, autoLooper;
	// private Processor processor;
	private final SendableChooser<String> mTrajectoryChooser = new SendableChooser<String>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		Registers.kReal.set(RobotBase.isReal());
		SmartDashboard.putBoolean("isReal", Registers.kReal.get());

		if (!Registers.kReal.get()) {
			Registers.kTelemetry.set(true);
		} else {
			Registers.kTelemetry.set(false);
		}

		// Get loopers
		subsystemLooper = SubsystemLooper.getInstance();
		teleopLooper = TeleopLooper.getInstance();
		autoLooper = AutoLooper.getInstance();
		// processor = Processor.getInstance();

		// Assign auto paths
		mTrajectoryChooser.setDefaultOption("None", null);
		for (final File fileEntry : Filesystem.getDeployDirectory().listFiles()) {
			if (!fileEntry.isDirectory()) {
				String filePath = fileEntry.getName();
				String[] splitPath = filePath.split(Pattern.quote("."));
				if (splitPath.length == 3) {
					if (splitPath[1].equals("wpilib")) {
						mTrajectoryChooser.addOption(splitPath[0], filePath);
					}
				}
			}
		}

		SmartDashboard.putData("TrajectoryChooser", mTrajectoryChooser);
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
		teleopLooper.stop();
		subsystemLooper.reset();
		autoLooper.reset();
		// processor.reset();

		Trajectory trajectory = new Trajectory();

		if (mTrajectoryChooser.getSelected() != null) {
			try {
				Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
						.resolve(mTrajectoryChooser.getSelected());
				trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			} catch (IOException ex) {
				DriverStation.reportError("Unable to open trajectory: " + mTrajectoryChooser.getSelected(),
						ex.getStackTrace());
				return;
			}

			Constants.kDriveHelper.resetGyro();

			Registers.kDriveAutoTrajectory.set(trajectory);
			Registers.kDriveAutoMode.set(DriveAutoMode.kTrajectory);
			Registers.kDriveMode.set(DriveMode.kDifferential);
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Registers.kDriveLV.set(Registers.kDriveAutoLV.get());
		Registers.kDriveRV.set(Registers.kDriveAutoRV.get());

		// System.out.println(Registers.kDriveAutoLV.get() + "," +
		// Registers.kDriveAutoRV.get());
	}

	/**
	 * This function is called once when teleop is enabled.
	 */
	@Override
	public void teleopInit() {
		teleopLooper.start();
		subsystemLooper.reset();
		autoLooper.reset();
		// processor.reset();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
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
	}

	/**
	 * This function is called once when the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		subsystemLooper.stop();
		teleopLooper.stop();
		autoLooper.stop();
		// processor.stop();
	}

	/**
	 * This function is called periodically when disabled.
	 */
	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This function is called once when test mode is enabled.
	 */
	@Override
	public void testInit() {
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
