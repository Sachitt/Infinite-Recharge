package org.frc.FRC.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

// Using WPILIB Docs and Cheif Delphi Forums
// Also uses some ode from trajectory generation tutorial by Green Hope Falcons

public class Drivetrain extends SubsystemBase {
  
  private static final double kGearRatio = 7.29;
  private static final double kWheelRadiusInches = 3.0;

  CANSparkMax left1= new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANSparkMax leftSlave1 = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave2 = new CANSparkMax(10,CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics, getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSlave1.follow(left1);
    leftSlave2.follow(left1);
    rightSlave1.follow(right1);
    rightSlave2.follow(right1);

    left1.setInverted(false);
    right1.setInverted(true);

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        left1.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        right1.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    left1.set(leftVolts / 12);
    right1.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getSpeeds());
  }
}
