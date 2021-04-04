
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {
    public static final XboxController driveController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    public static final int kDriveTrainLAMotor = 3;
    public static final int kDriveTrainLBMotor = 4;
    public static final int kDriveTrainLCMotor = 5;

    public static final int kDriveTrainRAMotor = 6;
    public static final int kDriveTrainRBMotor = 9;
    public static final int kDriveTrainRCMotor = 10;

    public static final double kDriveSlowPower = 0.4;
    public static final double kDriveRegularPower = 0.7;
    public static final double kDriveTurboPower = 0.95;

    public static final int kIntakeLMotor = 12;
    public static final int kIntakeRMotor = 13;

    public static final int kTubeLMotor = 14;
    public static final int kTubeRMotor = 15;

    public static final int kIntakeF = 4;
    public static final int kIntakeR = 5;

    public static final double kTriggerThreshold = 0.3;

    // Intake Constants
    public static final double kIntakeSpeed = 0.8;

    // Tube Constants
    public static final double kTubeIntakeSpeed = 0.7;
    public static final double kTubeShootSpeed = 1;

    // LimeLight
    public static final LimeLight kBackLL = new LimeLight("limelight-intake");
<<<<<<< HEAD
}
=======
}
>>>>>>> 2a8d4f38fa47d89597fc12bf6dc06f3ed33a5c19
