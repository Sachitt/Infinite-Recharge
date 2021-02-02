package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
    // USB Ports
    public static int DRIVER_PORT = 0;
    public static int OPERATOR_PORT = 1;

    // CANBUS Ports
    public static int[] SHOOTER_PORTS = { 1, 2 };
    public static int[] INTAKE_PORTS = { 12, 13, 14, 15 };
    public static int PIVOT_PORT = 11;
    public static int[] DRIVE_LEFT_PORTS = { 3, 4, 5 };
    public static int[] DRIVE_RIGHT_PORTS = { 6, 9, 10 };

    // Solenoid Ports
    public static int[] PNEUMATIC_SHOOTER_PORT = { 1, 0 };
    public static int[] PNEUMATIC_INTAKE_PORT = { 4, 5 };

    // DIO Ports
    public static int PIVOT_LIMIT_PORT = 1;
    public static int CLIMB_LOWER_LIMIT_PORT = 0;
    public static int CLIMB_UPPER_LIMIT_PORT = 2;

    // LimeLight Ports
    public static int PORT_PIPELINE = 0;

    // LimeLight Constants
    public static double MOUNTING_HEIGHT = 0.4191; // Meters
    public static double MOUNTING_ANGLE = 0.69; // Radians
    public static double REFRENCE_HEIGHT = 0.249; // Meters

    // Motor Constants
    public static int NEO_MAX_CURRENT = 45;
    public static int TALON_MAX_CURRENT = 60;

    // Controller Constants
    public static double TRIGGER_THRESHOLD = 0.3;

    // Drivetrain Constants
    public static double DRIVE_SLOW_POWER = 0.4;
    public static double DRIVE_REGULAR_POWER = 0.7;
    public static double DRIVE_TURBO_POWER = 0.9;

    public static float DRIVE_KP = 0.08f;
    public static float DRIVE_KI = 0;
    public static float DRIVE_KD = 0.04f;
    public static double DRIVE_MAX_ROTATION_SPEED = 0.2;
    public static double DRIVE_ROTATION_TOLERANCE = 5;

    // Shooter Constants
    public static double SHOOTER_LOWER_SPEED = 0.7;
    public static double SHOOTER_UPPER_SPEED = 0.9;
    public static double SHOOTER_TARGET_RPM = 4500;
    public static double SHOOTER_THRESHOLD_RPM = 100;

    public static double SHOOTER_TURN_SPEED = 0.2;

    public static double SHOOTER_SHOOT_TIME = 700;
    public static double SHOOTER_ROTATE_TIME = 2000;

    public static double SHOOTER_KP = 0.3;
    public static double SHOOTER_KI = 0;
    public static double SHOOTER_KD = 3.5;
    public static double SHOOTER_KF = 0.5;

    public static double SHOOTER_MAX_VELOCITY = 10; // Meters/second

    // Intake Constants
    public static double INTAKE_INTAKE_SPEED = 0.8;
    public static double INTAKE_TUBE_SPEED = 0.7;
    public static double INTAKE_SHOOT_SPEED = 1;

    // Pivot Constants
    public static double PIVOT_TELEOP_SPEED = 0.25;
    public static double PIVOT_AUTO_SPEED = 0.2;
    public static double PIVOT_ZERO_SPEED = 0.1;

    public static double PIVOT_ZERO_THRESHOLD = 5;
    public static double PIVOT_MAX_REVOLUTION = 83;
    public static double PIVOT_THRESHOLD = 1;

    public static double RADIANS_TO_REV = 10; // NEED TO GET

    // Auto Constants
    public static double AUTO_SHOOT_TIME = 2000;
    public static double AUTO_MOVE_TIME = 1500;
    public static double DRIVE_FORWARD_SPEED = 0.5;

    // Climb Constants
    public static final int SCREW_TALON_L = 8;
    public static final int SCREW_TALON_R = 7;
    public static final double SCREW_SPEED = 0.975;

    // solenoids
    public static final int PISTONL_FWD = 2;
    public static final int PISTONL_BKWD = 3;

    public static double WHEELBASE_WIDTH = 0.6898513;
    public static double WHEEL_DIAMETER = 0.1524;
    public static double MAX_VELOCITY = 2;
    static final double MAX_ACCELERATION = 0.5; // Not verified

    public static final int ENCODER_REV = 42;// temporary

    public static final int WHEEL_REV = 7;
}