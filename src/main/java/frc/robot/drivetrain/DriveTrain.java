package frc.robot.drivetrain;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PIDControl;
import frc.robot.Robot;

public class DriveTrain {
    private CANSparkMax lsparkA, lsparkB, lsparkC, rsparkA, rsparkB, rsparkC;
    private DifferentialDrive drive;

    private boolean invert;

    private PIDControl pidControl;
    private Alignment alignment;

    public DriveTrain() {
        lsparkA = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[0], MotorType.kBrushless);
        lsparkB = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[1], MotorType.kBrushless);
        lsparkC = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[2], MotorType.kBrushless);

        rsparkA = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[0], MotorType.kBrushless);
        rsparkB = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[1], MotorType.kBrushless);
        rsparkC = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[2], MotorType.kBrushless);

        // Group sparks into an ArrayList for a cleaner intialization loop
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

        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setIdleMode(IdleMode.kBrake); // IdleMode will always be kBrake for driveTrain motors

        }

        SpeedControllerGroup leftGroup = new SpeedControllerGroup(lsparkA, lsparkB, lsparkC);
        SpeedControllerGroup rightGroup = new SpeedControllerGroup(rsparkA, rsparkB, rsparkC);

        drive = new DifferentialDrive(leftGroup, rightGroup);

        pidControl = new PIDControl(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KP);
        pidControl.setMaxSpeed(Constants.DRIVE_MAX_ROTATION_SPEED);
        pidControl.setTolerance(Constants.DRIVE_ROTATION_TOLERANCE);
        alignment = new Alignment();

        // Store variables
        this.invert = false;
    }

    public void run() {
        if (Robot.driverController.getAButtonPressed()) {
            invert = !invert;
        }
        if (Robot.driverController.getBButton()) {
            align();
        } else {
            runTankDrive();

        }

        System.out.println(lsparkA.getEncoder().getVelocity() + " " +lsparkB.getEncoder().getVelocity()+ " " + lsparkC.getEncoder().getVelocity() + "  ,  " + rsparkA.getEncoder().getVelocity() + " " +rsparkB.getEncoder().getVelocity()+ " " + rsparkC.getEncoder().getVelocity());
    }

    public void moveForward() {
        drive.arcadeDrive(Constants.DRIVE_FORWARD_SPEED, 0);
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
        double drivePower = Constants.CLUBD_POWER;
       

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
