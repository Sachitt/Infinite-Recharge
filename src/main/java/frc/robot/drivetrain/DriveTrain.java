package frc.robot.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Limelight;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.controller.*;

public class DriveTrain {

    private CANSparkMax lMotorA, lmotorB, lmotorC, rmotorA, rmotorB, rmotorC;
    private final int RB_PORT = 6, LB_PORT = 3, LF_PORT = 4, RF_PORT = 9;
    public double Adjust;
    private boolean switch1;
    DifferentialDrive testDrive;
    SpeedControllerGroup controllerL, controllerR;
    public static double leftD;
    public static double rightD;
    Limelight Lime;
    PIDController drivePID;

    int R = 1;
    int L = 1;

    double kP = 0.08;
    double speed = 0.9;
    double min = 0.05;

    public DriveTrain() {
        lMotorA = new CANSparkMax(LB_PORT, MotorType.kBrushless);
        rmotorA = new CANSparkMax(RB_PORT, MotorType.kBrushless);
        lmotorB = new CANSparkMax(LF_PORT, MotorType.kBrushless);
        rmotorB = new CANSparkMax(RF_PORT, MotorType.kBrushless);
        lmotorC = new CANSparkMax(5, MotorType.kBrushless);
        rmotorC = new CANSparkMax(10, MotorType.kBrushless);

        controllerL = new SpeedControllerGroup(lMotorA, lmotorB, lmotorC);
        controllerR = new SpeedControllerGroup(rmotorA, rmotorB, rmotorC);

        testDrive = new DifferentialDrive(controllerL, controllerR);

        this.switch1 = false;

        Lime = new Limelight();

        drivePID = new PIDController(-(0.2605), 0, 0.069);
        drivePID.setTolerance(0.5);

    }

    public void TankDrive() {

        if (Robot.driverController.getAButtonPressed()) {
            switch1 = !switch1;
        }
        // constants

        double axisR = Robot.driverController.getY(Hand.kRight);
        double axisL = Robot.driverController.getY(Hand.kLeft);

        if (axisR < 0) {
            R = 1;
        } else if (axisR > 0) {
            R = -1;
        }

        if (axisL < 0) {
            L = 1;
        } else if (axisL > 0) {
            L = -1;
        }

        double power = 0.8;

        leftD = L * power * Math.pow(Math.abs(axisL), 1.5);
        rightD = R * power * Math.pow(Math.abs(axisR), 1.5);

        if (switch1) {
            testDrive.tankDrive(-rightD, -leftD);
        } else {
            testDrive.tankDrive(leftD, rightD);
        }

    }

    public void AlignDriveTrain() {
        Lime.selectPipeline(0);
        if (Lime.targetFound()) {
            double xError = Lime.getTx();
            if (Lime.getTx() > 1.0) {
                Adjust = (xError * kP * speed) - min;
            } else if (Lime.getTx() < 1.0) {
                Adjust = (xError * kP * speed) + min;
            }
            leftD += Adjust;
            rightD -= Adjust;

            System.out.println(Adjust);
            System.out.println(leftD + ", " + rightD);
        } else {
            System.out.println("Target not found");
        }

    }

    public void Align() {

        double Adjust = MathUtil.clamp(drivePID.calculate((Lime.getXError() * 0.3), 0), -0.26, 0.26);

        testDrive.arcadeDrive(0, Adjust);

        // System.out.println(Adjust);
    }

    public void run() {
        if (Robot.driverController.getBButton()) {
            Align();
        } else {
            TankDrive();
        }
    }
}
