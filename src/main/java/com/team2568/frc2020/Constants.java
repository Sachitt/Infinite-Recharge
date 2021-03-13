package com.team2568.frc2020;

import com.team2568.frc2020.fsm.auto.Pivot.PivotAlgorithmMode;
import com.team2568.lib.DifferentialDriveHelper;
import com.team2568.lib.limelight.Distance;
import com.team2568.lib.limelight.LimeLight;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

// All physical constants are in MKS and radians
public class Constants {
	// RunnableConstants
	public static final double kDefaultPeriod = 0.01;

	// Controllers
	public static final Controller kDriveController = new Controller(0);
	public static final Controller kOperatorController = new Controller(1);

	// CAN Ports

	// https://docs.google.com/spreadsheets/d/1xZ8lbKK4TP_i9pDaXgZNcLdZUCCJ9CWRHAVqdm4gPFw/edit?usp=sharing

	public static final int kShooterLMotor = 1;
	public static final int kShooterRMotor = 2;

	public static final int kPivotMotor = 11;

	public static final int kIntakeLMotor = 12;
	public static final int kIntakeRMotor = 13;

	public static final int kTubeLMotor = 14;
	public static final int kTubeRMotor = 15;

	public static final int kClimbLMotor = 8;
	public static final int kClimbRMotor = 7;

	public static final int kDriveTrainLAMotor = 3;
	public static final int kDriveTrainLBMotor = 4;
	public static final int kDriveTrainLCMotor = 5;

	public static final int kDriveTrainRAMotor = 6;
	public static final int kDriveTrainRBMotor = 9;
	public static final int kDriveTrainRCMotor = 10;

	// Solenoid Ports
	public static final int kShooterF = 1;
	public static final int kShooterR = 0;
	public static final int kIntakeF = 4;
	public static final int kIntakeR = 5;
	public static final int kClimbF = 2;
	public static final int kClimbR = 3;

	// Digital Ports
	public static final int kPivotLimit = 1;
	public static final int kClimbLLimit = 0;
	public static final int kClimbULimit = 2;

	// LimeLight Objects
	public static final LimeLight kFrontLL = new LimeLight("limelight");
	public static final LimeLight kBackLL = new LimeLight("lemontorch");

	// Controller Constants
	public static final double kTriggerThreshold = 0.3;
	public static final double kJoystickDeadzone = 0.1;

	// SparkMax Constants
	public static final int kSparkCurrentLimit = 40;

	// TalonSRX Constants
	public static final int kTalonSRXCurrentLimit = 60;

	// Shooter Constants
	public static final double kShooterRPM = 4500;
	public static final double kShooterLowRPM = 3500;
	public static final double kShooterTurnSpeed = 500;
	public static final double kShooterSpinTime = 0.7;

	public static final double kShooterkP = 0.3;
	public static final double kShooterkI = 0;
	public static final double kShooterkD = 3.5;
	public static final double kShooterkF = 0.5;

	// Intake Constants
	public static final double kIntakeSpeed = 0.8;

	// Tube Constants
	public static final double kTubeIntakeSpeed = 0.7;
	public static final double kTubeShootSpeed = 1;

	// Pivot Constants
	public static final double kPivotAutoMinSpeed = -0.2;
	public static final double kPivotAutoMaxSpeed = 0.25;

	public static final double kPivotZeroSpeed = 0.05;
	public static final double kPivotTeleopSpeed = 0.25;

	public static final double kPivotZeroThreshold = 5;
	public static final double kPivotHighestThreshold = 80;
	public static final double kPivotTargetRevThreshold = 0.5;

	public static final double kPivotTrench = 3.8;
	public static final double kPivotLine = 16.35;
	public static final double kPivotAgainst = 79;
	public static final double kPivotWheel = 33;

	public static final double kPivotkP = 0.6;
	public static final double kPivotkI = 0;
	public static final double kPivotkD = 4;
	public static final double kPivotkF = 0;

	public static final PivotAlgorithmMode kPivotAlgorithmMode = PivotAlgorithmMode.kIterate;

	// Climb Constants
	public static final double kClimbSpeed = 0.975;

	// Drive Constants
	public static final double kDriveSlowPower = 0.4;
	public static final double kDriveRegularPower = 0.7;
	public static final double kDriveTurboPower = 0.95;

	public static final double kDriveAlignkP = 0.28;
	public static final double kDriveAlignkI = 0;
	public static final double kDriveAlignkD = 0.055;

	public static final double kDriveMaxRotationSpeed = 0.23;

	public static final double kVolt = 0.179;
	public static final double kVoltSecondPerMeter = 2.71;
	public static final double kVoltSecondSquaredPerMeter = 0.364;

	public static final double kDriveVelocitykP = 2.3;
	public static final double kDriveVelocitykI = 0;
	public static final double kDriveVelocitykD = 0;

	public static final double kDrivekB = 2.0;
	public static final double kDrivekZeta = 0.7;

	// LimeLight Constants
	public static final int kUpperPort = 1;

	// Physical Constants

	// Motor Constants
	public static final double kTicksToRevolution = 42;

	// Shooter Constants
	public static final double kShooterVelocity = 30.48;
	public static final double kShooterLowVelocity = 25;

	// Pivot Constants
	public static final double kPivotToLimeLight = 0.7408926; // The horizontal distance from the pivot point of the
																// tube to the limelight
	public static final double kTubeLength = 0.861401122; // The length of the tube from the pivot point to the exit
															// point
	public static final double kPivotHeight = 0.1676908; // The height of the pivot point of the tube off the ground

	// DriveTrain Constants
	public static final double kTrackWidth = 0.53975;
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

	public static final double kMaxSpeed = 0;
	public static final double kMaxAcceleration = 0;

	public static final double kWheelDiameter = 0.1524;
	public static final double kDistancePerRevolution = Math.PI * kWheelDiameter;
	public static final double kGearReduction = 10.3846153846;

	public static final double kMaxVoltage = 7;

	public static final DifferentialDriveHelper kDriveHelper = new DifferentialDriveHelper(
			kDistancePerRevolution / kGearReduction);

	// Upper Port Constants
	public static final double kUpperPortHeight = 2.1425; // Height to the center of the port for theta calculations
	public static final double kUpperPortHeightDistance = 2.1425; // Lower than actual height b/c bounding box is lower

	// Forward Facing Constants
	public static final double kForwardLLMountingHeight = 0.4191;
	public static final double kForwardLLMountingAngle = 0.5759;

	public static final Distance kForwardDistance = new Distance(kForwardLLMountingHeight, kForwardLLMountingAngle,
			kUpperPortHeightDistance);

	// Physics
	public static final double kG = 9.81; // Gravity Constant in MKS units
}
