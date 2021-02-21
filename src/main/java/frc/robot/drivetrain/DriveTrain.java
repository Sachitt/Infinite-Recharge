package frc.robot.drivetrain;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import frc.robot.PIDControl;
import frc.robot.Robot;

public class DriveTrain {
    private CANSparkMax lsparkA, lsparkB, lsparkC, rsparkA, rsparkB, rsparkC;
    private DifferentialDrive drive;
    private SpeedControllerGroup leftGroup, rightGroup;

    private boolean invert;

    private PIDControl pidControl;
    private Alignment alignment;

    private AHRS mGyro;

    private final DifferentialDriveOdometry mOdometry;
    private DifferentialDriveVoltageConstraint mConstraint;
    private TrajectoryConfig mConfig;
    private Trajectory mTrajectory;
    private RamseteController mRamesete;

    private boolean mStarted;

    public DriveTrain() {
        lsparkA = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[0], MotorType.kBrushless);
        lsparkB = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[1], MotorType.kBrushless);
        lsparkC = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[2], MotorType.kBrushless);

        rsparkA = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[0], MotorType.kBrushless);
        rsparkB = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[1], MotorType.kBrushless);
        rsparkC = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[2], MotorType.kBrushless);

        // Group sparks into an ArrayList for a cleaner intialization loop
        ArrayList<CANSparkMax> sparkList = new ArrayList<CANSparkMax>() {
            /**
             *
             */
            private static final long serialVersionUID = 1L;

            {
                add(lsparkA);
                add(lsparkB);
                add(lsparkC);
                add(rsparkA);
                add(rsparkB);
                add(rsparkC);
            }
        };

        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setIdleMode(IdleMode.kBrake); // IdleMode will always be kBrake for driveTrain motors

        }

        leftGroup = new SpeedControllerGroup(lsparkA, lsparkB, lsparkC);
        rightGroup = new SpeedControllerGroup(rsparkA, rsparkB, rsparkC);

        drive = new DifferentialDrive(leftGroup, rightGroup);

        pidControl = new PIDControl(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KP);
        pidControl.setMaxSpeed(Constants.DRIVE_MAX_ROTATION_SPEED);
        pidControl.setTolerance(Constants.DRIVE_ROTATION_TOLERANCE);
        alignment = new Alignment();

        // Store variables
        this.invert = false;

        // Odometry
        lsparkA.getEncoder().setVelocityConversionFactor(Constants.DRIVE_ENCODER_DISTANCE_PER_PULSE);
        rsparkA.getEncoder().setVelocityConversionFactor(Constants.DRIVE_ENCODER_DISTANCE_PER_PULSE);
        lsparkA.getEncoder().setPositionConversionFactor(Constants.DRIVE_ENCODER_DISTANCE_PER_PULSE);
        rsparkA.getEncoder().setPositionConversionFactor(Constants.DRIVE_ENCODER_DISTANCE_PER_PULSE);

        mGyro = new AHRS();
        mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());
        mConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DRIVE_VOLTS, Constants.DRIVE_VOLTS_SECOND_METER,
                        Constants.DRIVE_VOLTS_SECOND_SQUARED_METER),
                Constants.DRIVE_KINEMATICS, Constants.DRIVE_MAX_VOLTAGE);
        mConfig = new TrajectoryConfig(Constants.DRIVE_MAX_VELOCITY, Constants.DRIVE_MAX_ACCELERATION)
                .setKinematics(Constants.DRIVE_KINEMATICS).addConstraint(mConstraint);
        mTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                mConfig);
    }

    public void run() {
        if (Robot.driverController.getXButtonPressed()) {
            invert = !invert;
        }

        if (Robot.driverController.getBButton()) {
            align();
        } else if (Robot.driverController.getAButton()) {
            forward();
        } else if (Robot.driverController.getYButton()) {
            backward();
        } else {
            runTankDrive();
            reset();
            mOdometry.update(mGyro.getRotation2d(), lsparkA.getEncoder().getPosition(),
                    rsparkA.getEncoder().getPosition());
            mStarted = false;
        }
    }

    public void resetEncoders() {
        lsparkA.getEncoder().setPosition(0);
        rsparkA.getEncoder().setPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, mGyro.getRotation2d());
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(-rightVolts);
        drive.feed();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(lsparkA.getEncoder().getVelocity(), rsparkA.getEncoder().getVelocity());
    }

    public void followTrajectory() {
        if (!mStarted) {
            resetOdometry(mTrajectory.getInitialPose());
        }

        mRamesete = new RamseteController(Constants.DRIVE_RAMSETE_B, 
                        Constants.DRIVE_RAMSETE_ZETA);

        
    }

    public void moveForward() {
        drive.arcadeDrive(Constants.DRIVE_FORWARD_SPEED, 0);
    }

    public void forward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        double driveSpeed = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getBumper(Hand.kLeft))
            driveSpeed = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getBumper(Hand.kRight))
            driveSpeed = Constants.DRIVE_TURBO_POWER;

        drive.arcadeDrive(driveSpeed, 0);
    }

    public void backward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        double driveSpeed = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getBumper(Hand.kLeft))
            driveSpeed = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getBumper(Hand.kRight))
            driveSpeed = Constants.DRIVE_TURBO_POWER;

        drive.arcadeDrive(-driveSpeed, 0);
    }

    public void stop() {
        drive.arcadeDrive(0, 0);
    }

    public void align() {
        if (alignment.targetFound()) {
            drive.arcadeDrive(0, pidControl.getValue(0, alignment.getError()));
        }
    }

    public void reset() {
        pidControl.cleanup();
    }

    private void runTankDrive() {
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
        double drivePower = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getBumper(Hand.kLeft))
            drivePower = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getBumper(Hand.kRight))
            drivePower = Constants.DRIVE_TURBO_POWER;

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
    }
}
