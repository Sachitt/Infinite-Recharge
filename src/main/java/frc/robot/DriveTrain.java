package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.json.*;
import java.io.FileWriter;
import com.revrobotics.CANSparkMax.IdleMode;
 
/**
 * Modified basic tank drive drivetrain for recording
 */
public class DriveTrain {
    private CANSparkMax laMotor, lbMotor, lcMotor, raMotor, rbMotor, rcMotor;
    private SpeedControllerGroup lGroup, rGroup;

    private DifferentialDrive drive;

    private boolean invert;

    // Modified
    private final JoystickMap map = new JoystickMap();

    private ArrayList<TankValue> recording;

    private int i, j;
    private boolean toggle, active, record;

    private double lSpeed, rSpeed;

    public JSONObject jsonObj;

    public static List<String> tankVals = new ArrayList<String>();

    // FileWriter file = new FileWriter("src/main/deploy/tankvals.json");

    /**
     * Class that manages setting and getting of four tank value lists
     * 
     */
    private class JoystickMap {
        private List<TankValue> mUp, mRight, mDown, mLeft;

        public JoystickMap() {
            mUp = mRight = mDown = mLeft = new ArrayList<>();
        }

        public void reset() {
            mUp = mRight = mDown = mLeft = new ArrayList<>();
        }

        public void set(int key, List<TankValue> value) {
            switch (key) {
            case 0:
                mUp = value;
                break;
            case 90:
                mRight = value;
                break;
            case 180:
                mDown = value;
                break;
            case 270:
                mLeft = value;
                break;
            default:
                return;
            }
        }

        public List<TankValue> get(int key) {
            switch (key) {
            case 0:
                return mUp;
            case 90:
                return mRight;
            case 180:
                return mDown;
            case 270:
                return mLeft;
            default:
                return new ArrayList<TankValue>();
            }
        }
    }

    public class TankValue {
        private double mL, mR;

        public TankValue(double l, double r) {
            mL = l;
            mR = r;
        }

        public double getL() {
            return mL;
        }

        public double getR() {
            return mR;
        }

        @Override
        public String toString() {
            // return "{mL: " + mL + ", mR:" + mR + "}";
            return mL + "!" + mR;
            }
    
    }

    public DriveTrain() {
        laMotor = new CANSparkMax(Constants.kDriveTrainLAMotor, MotorType.kBrushless);
        lbMotor = new CANSparkMax(Constants.kDriveTrainLBMotor, MotorType.kBrushless);
        lcMotor = new CANSparkMax(Constants.kDriveTrainLCMotor, MotorType.kBrushless);

        raMotor = new CANSparkMax(Constants.kDriveTrainRAMotor, MotorType.kBrushless);
        rbMotor = new CANSparkMax(Constants.kDriveTrainRBMotor, MotorType.kBrushless);
        rcMotor = new CANSparkMax(Constants.kDriveTrainRCMotor, MotorType.kBrushless);

        laMotor.setIdleMode(IdleMode.kBrake);
        lbMotor.setIdleMode(IdleMode.kBrake);
        lcMotor.setIdleMode(IdleMode.kBrake);
        raMotor.setIdleMode(IdleMode.kBrake);
        rbMotor.setIdleMode(IdleMode.kBrake);
        rcMotor.setIdleMode(IdleMode.kBrake);

        lGroup = new SpeedControllerGroup(laMotor, lbMotor, lcMotor);
        rGroup = new SpeedControllerGroup(raMotor, rbMotor, rcMotor);

        drive = new DifferentialDrive(lGroup, rGroup);

        // Modified
        i = j = 0;

        invert = toggle = active = record = false;

        jsonObj = new JSONObject();
    }

    public void forward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        double driveSpeed = Constants.kDriveRegularPower;
        if (Constants.driveController.getBumper(Hand.kLeft))
            driveSpeed = Constants.kDriveSlowPower;
        else if (Constants.driveController.getBumper(Hand.kRight))
            driveSpeed = Constants.kDriveTurboPower;

        drive.arcadeDrive(driveSpeed, 0);
    }

    public void backward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        double driveSpeed = Constants.kDriveRegularPower;
        if (Constants.driveController.getBumper(Hand.kLeft))
            driveSpeed = Constants.kDriveSlowPower;
        else if (Constants.driveController.getBumper(Hand.kRight))
            driveSpeed = Constants.kDriveTurboPower;

        drive.arcadeDrive(-driveSpeed, 0);
    }

    

    public void run() {
        if (Constants.driveController.getXButtonPressed()) {
            invert = !invert;
        }
        // } else if (Constants.driveController.getYButton()) {
        //     backward();
        // } else if (Constants.driveController.getAButton()) {
        //     forward();
        // } 

        // Modified

        // Reset flags
        active = false;

        if (Constants.driveController.getPOV() != -1) {
            // Cache which dpad was pressed for recording
            i = Constants.driveController.getPOV();

            // Recording already set
            if (map.get(i).size() != 0) {
                if (j < map.get(i).size() - 1) {
                    // Switch from controllers to map to get motor speeds
                    active = true;

                    lSpeed = map.get(i).get(j).getL();
                    rSpeed = map.get(i).get(j).getR();

                    // increment every cycle run is called
                    j++;
                }
            }
        }

        // Reset all recorded values if B is pressed
        if (Constants.driveController.getBButton()) {
            map.reset();
        } 

        if (Constants.driveController.getStartButtonPressed()) {
            // Recording not set
            if (map.get(i).size() == 0) {
                // Toggle starts in the false position
                if (toggle) {
                    // update the map value and turn off recording
                    map.set(i, recording);
                    
                    for(int x=0; x<recording.size(); x++) {
                        System.out.println("tankValues.add(\"" + recording.get(x).toString() + "\");");
                    }
        
                    //System.out.println(recording);

                    //buildJsonFile(recording);

                    record = false;
                } else {
                    // Reset recording arraylist and turn on functionality
                    recording = new ArrayList<TankValue>();
                    record = true;
                }

                // switch toggle everytime it enters this section
                toggle = !toggle;
            }
        }

        if (!active) {
            setSpeeds();
            j = 0;
        } 

        if (record) {
            recording.add(new TankValue(lSpeed, rSpeed));
        }

        tankDrive(lSpeed, rSpeed);
        }
    

    // private void buildJsonFile(ArrayList<TankValue> infolist) {
    //     for (int z = 0; z<infolist.size(); z++) {
    //         JSONObject tempObj = new JSONObject();
    //         tempObj.put("mL", infolist.get(z).getL());
    //         tempObj.put("mR", infolist.get(z).getR());

    //         JSONArray tempArray = new JSONArray();
    //         tempArray.put(tempObj);

    //         jsonObj.put("tankvalues", tempArray);
    //     }

    //     file.write(jsonObj.toJSONString());
    //     file.close();
    // }

    private void setSpeeds() {
        // constants to easily configure if drive is opposite
        int constR = 1, constL = 1;

        // Get vertical value of the joysticks
        double rAxis = Constants.driveController.getY(Hand.kRight);
        double lAxis = Constants.driveController.getY(Hand.kLeft);

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
        double drivePower = Constants.kDriveRegularPower;
        if (Constants.driveController.getBumper(Hand.kLeft))
            drivePower = Constants.kDriveSlowPower;
        else if (Constants.driveController.getBumper(Hand.kRight))
            drivePower = Constants.kDriveTurboPower;
        else if (Constants.driveController.getTriggerAxis(Hand.kRight) > 0.4) {
            drivePower = 1;
        }

        // However driveExponent should be constant (Changeable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("Drive Exponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        if (invert) {
            lSpeed = -driveR;
            rSpeed = -driveL;
        } else {
            lSpeed = driveL;
            rSpeed = driveR;
        }
    }

    public void tankDrive(double lSpeed, double rSpeed) {
        drive.tankDrive(lSpeed, rSpeed);
    }

    public void stop() {
        drive.stopMotor();
    }

    public ArrayList<String> threeTrench() {
    
    ArrayList<String> tankValues = new ArrayList<String>();
    
    return tankValues;
    }

}