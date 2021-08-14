package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    CANSparkMax lsparkA = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax lsparkB = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax lsparkC = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax rsparkA = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax rsparkB = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax rsparkC = new CANSparkMax(10, MotorType.kBrushless);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(lsparkA, lsparkB, lsparkC);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rsparkA, rsparkB, rsparkC);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    private final CANEncoder leftEncoder = lsparkA.getEncoder();
    private final CANEncoder rightEncoder = rsparkA.getEncoder();

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final DifferentialDriveOdometry odometry;

    private boolean invert;

    public DriveSubsystem() {
        ArrayList<CANSparkMax> sparkList = new ArrayList<CANSparkMax>() {
            {
                add(lsparkA);
                add(lsparkB);
                add(lsparkC);
                add(rsparkA);
                add(rsparkB);
                add(rsparkC);
            }
        };

        // Voltage Regulation
        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(45);
            spark.setSecondaryCurrentLimit(45);
            spark.setIdleMode(IdleMode.kBrake);
        }

        // Velocity Conversions
        leftEncoder.setVelocityConversionFactor(0.25);
        rightEncoder.setVelocityConversionFactor(0.25);

        // Position Conversions
        leftEncoder.setPositionConversionFactor(0.25);
        rightEncoder.setPositionConversionFactor(0.25);

        // Sets the robot Position in a 2D Space
        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        this.invert = false;
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(),
                leftEncoder.getPosition() / Constants.kDriveGearRatio * 2 * Math.PI * Units.inchesToMeters(3.0),
                rightEncoder.getPosition() / Constants.kDriveGearRatio * 2 * Math.PI * Units.inchesToMeters(3.0));

        System.out.println(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // returns in RPM, divide by gear ratio multiply by circumference and /60 to get
        // Meters per second
        // RPM --> MPS (meters per second)
        return new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity() / Constants.kDriveGearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
                rightEncoder.getVelocity() / Constants.kDriveGearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(-rightVolts);
        drive.feed();
    }

    public CANEncoder getLeftCanEncoder() {
        return leftEncoder;
    }

    public CANEncoder getRightCanEncoder() {
        return rightEncoder;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    private void MasterCodeRunTankDrive() {
        // constants to easily configure if drive is opposite
        int constR = 1, constL = 1;

        // Get vertical value of the joysticks
        double rAxis = Robot.driverController.getY(Hand.kRight);
        double lAxis = Robot.driverController.getY(Hand.kLeft);

        // Use a constant multiplier for +/- direction as the driveExponent could be
        // even and negate the sign
        if (rAxis < 0) {
            constR *= 1;
        } else if (rAxis > 0) {
            constR *= -1;
        }

        if (lAxis < 0) {
            constL *= 1;
        } else if (lAxis > 0) {
            constL *= -1;
        }

        // LB and RB are used to change the drivePower during the match
        double drivePower = 0.6;
        if (Robot.driverController.getBumper(Hand.kLeft))
            drivePower = 0.3;
        else if (Robot.driverController.getBumper(Hand.kRight))
            drivePower = 0.9;

        // However driveExponent should be constant (Changeable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("Drive Exponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        // Our drivers prefer tankDrive
        // invert will switch R and L
        if (invert) {
            drive.tankDrive(-driveR, -driveL);
        } else {
            drive.tankDrive(driveL, driveR);
        }

        /*
         * public Command runTrajectory(){ var autoVoltageConstraint = new
         * DifferentialDriveVoltageConstraint( new
         * SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSeconds,
         * Constants.kaVoltSecondsSquaredPerMeter), Constants.DRIVE_KINEMATICS, 10);
         * 
         * TrajectoryConfig config = new TrajectoryConfig(Constants.maxSpeedMPS,
         * Constants.maxAccelerationMPSsq) .setKinematics(Constants.DRIVE_KINEMATICS)
         * .addConstraint(autoVoltageConstraint);
         * 
         * 
         * Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory( new
         * Pose2d(0,0, new Rotation2d(0)), List.of( new Translation2d(1,1), new
         * Translation2d(2,-1) ), new Pose2d(3,0, new Rotation2d(0)), config
         * 
         * );
         * 
         * 
         * RamseteCommand ramseteCommand = new RamseteCommand( testTrajectory,
         * drive1::getPose, new RamseteController(Constants.RamseteB,
         * Constants.RamseteZeta), new SimpleMotorFeedforward(Constants.ksVolts,
         * Constants.kvVoltSeconds, Constants.kaVoltSecondsSquaredPerMeter),
         * Constants.DRIVE_KINEMATICS, drive1::getWheelSpeeds, new
         * PIDController(Constants.kPDriveVel,0, 0), new
         * PIDController(Constants.kPDriveVel,0, 0), drive1::tankDriveVolts, drive1
         * 
         * );
         * 
         * drive1.resetOdometry(testTrajectory.getInitialPose());
         * 
         * return ramseteCommand.andThen(() -> drive1.tankDriveVolts(0, 0));
         * 
         * }
         */

    }
}
