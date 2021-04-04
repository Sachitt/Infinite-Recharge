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

    public void run() {
        if (Constants.driveController.getXButtonPressed()) {
            invert = !invert;
        }

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

    public ArrayList<String> buildARED() {
    
    ArrayList<String> tankValues = new ArrayList<String>();
    tankValues.add("-0.0!-0.002042989348916705");
    tankValues.add("-0.0!-0.009877484060721586");
    tankValues.add("-0.0!-0.173641574676586");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-0.0!-0.7");
    tankValues.add("-3.9262150805562714E-4!-0.7");
    tankValues.add("-0.013036186316018142!-0.7");
    tankValues.add("-0.28041639383088796!-0.7");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.9366823948417873!-0.95");
    tankValues.add("-0.7834328403160343!-0.95");
    tankValues.add("-0.7712130899609478!-0.95");
    tankValues.add("-0.7470300992813955!-0.95");
    tankValues.add("-0.7350671619944898!-0.95");
    tankValues.add("-0.7350671619944898!-0.95");
    tankValues.add("-0.3805651059133479!-0.95");
    tankValues.add("-0.34572183395333417!-0.95");
    tankValues.add("-0.7834328403160343!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.2805367115461971");
    tankValues.add("-0.95!0.12963599630516248");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.7!0.7");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.95");
    tankValues.add("-0.95!0.21063461947585355");
    tankValues.add("-0.95!-0.009654852869040276");
    tankValues.add("-0.36295763298183636!-0.5242805533653558");
    tankValues.add("-0.18773778018981926!-0.95");
    tankValues.add("-0.6996944350358608!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.6312857252546427");
    tankValues.add("-0.95!-0.02509306539395835");
    tankValues.add("-0.95!0.17742830254147007");
    tankValues.add("-0.95!0.5848789921387129");
    tankValues.add("-0.95!0.8578338356978426");
    tankValues.add("-0.95!0.26132560089061324");
    tankValues.add("-0.95!-0.030655057963271178");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.07399403468667116!-0.95");
    tankValues.add("0.03409815627550694!-0.95");
    tankValues.add("0.6745554688929472!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.95!-0.95");
    tankValues.add("0.3593304573411076!-0.95");
    tankValues.add("-0.18773778018981926!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.95!-0.95");
    tankValues.add("-0.5504432310494494!-0.7");
    tankValues.add("-0.24849553416718553!-0.7");
    tankValues.add("-0.005885137466266466!0.1307366439779253");
    tankValues.add("-0.0!3.982037371533506E-4");
    tankValues.add("-0.0!-0.0");
    tankValues.add("-0.0!-0.0");
    return tankValues;
    }

    public ArrayList<String> buildABLUE() {
        ArrayList<String> tankValues = new ArrayList<String>();
        
 
        return tankValues;
    }

    public ArrayList<String> buildBRED() {
        ArrayList<String> tankValues = new ArrayList<String>();
        tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-1.1275092010998849E-4!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-8.145886472909566E-4!-0.0");
tankValues.add("-0.004760823393025678!-0.0");
tankValues.add("-0.004760823393025678!-0.0");
tankValues.add("-0.003743661444161224!-0.0");
tankValues.add("-0.00283656242301508!-0.0");
tankValues.add("-0.00283656242301508!-0.0");
tankValues.add("-0.0013671874999999997!-0.0");
tankValues.add("-0.0013671874999999997!-0.0");
tankValues.add("-0.004760823393025678!-0.0");
tankValues.add("-0.06783324206055821!-0.0");
tankValues.add("-0.6804356756538082!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-0.0");
tankValues.add("-0.7!-1.1275092010998849E-4");
tankValues.add("-0.7!-1.1275092010998849E-4");
tankValues.add("-0.7!-0.18985635370073334");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.3279614588277281!-0.7");
tankValues.add("-0.11977172981022487!-0.7");
tankValues.add("-0.018489627132390364!-0.7");
tankValues.add("-0.004760823393025678!-0.7");
tankValues.add("-0.00283656242301508!-0.7");
tankValues.add("-0.002042989348916705!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-3.9262150805562714E-4!-0.7");
tankValues.add("-0.003743661444161224!-0.7");
tankValues.add("-0.016578149946207477!-0.7");
tankValues.add("-0.148056971728524!-0.7");
tankValues.add("-0.4569814459061834!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.6139098833510541!-0.7");
tankValues.add("-0.16837999142308177!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.003743661444161224!-0.7");
tankValues.add("-0.003743661444161224!-0.7");
tankValues.add("-0.21830050456521757!-0.7");
tankValues.add("-0.2870056058161617!-0.7");
tankValues.add("-0.3493451750095106!-0.7");
tankValues.add("-0.532876941442514!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.30038764856026656");
tankValues.add("-0.7!3.982037371533506E-4");
tankValues.add("-0.7!0.23956743318553675");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.7");
tankValues.add("-0.7!0.35431208392629043");
tankValues.add("-0.7!0.0013866259507418844");
tankValues.add("-0.7!-0.10240824234750336");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.3787865946591886");
tankValues.add("-0.7!-0.15807330850643883");
tankValues.add("-0.7!-0.07136160283048888");
tankValues.add("-0.7!-0.10663569883744035");
tankValues.add("-0.7!-0.2610581104010298");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.4170693874129698!-0.7");
tankValues.add("-0.25474240396561465!-0.7");
tankValues.add("-0.2010222121244811!-0.7");
tankValues.add("-0.12430054668137229!-0.7");
tankValues.add("-0.05772847109631412!-0.7");
tankValues.add("-0.05772847109631412!-0.7");
tankValues.add("-0.0749692554940743!-0.7");
tankValues.add("-0.08626369944065142!-0.7");
tankValues.add("-0.16319048564558547!-0.7");
tankValues.add("-0.38631198669026223!-0.7");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.8842495765540311!-0.95");
tankValues.add("-0.6650991583380599!-0.95");
tankValues.add("-0.5874252307709177!-0.95");
tankValues.add("-0.5982586824610991!-0.95");
tankValues.add("-0.6996944350358608!-0.95");
tankValues.add("-0.7834328403160343!-0.95");
tankValues.add("-0.845808222491916!-0.95");
tankValues.add("-0.9234484169587398!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.845808222491916!-0.95");
tankValues.add("-0.43559835017437365!-0.95");
tankValues.add("-0.27281585931179575!-0.95");
tankValues.add("-0.11185668711148385!-0.95");
tankValues.add("-0.02477274241085873!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
tankValues.add("-0.0!-0.7");
        return tankValues;
    }

    public ArrayList<String> buildBBLUE() {
        ArrayList<String> tankValues = new ArrayList<String>();
tankValues.add("-0.0013671874999999997!-3.9262150805562714E-4");
tankValues.add("-0.0824207168189881!-0.07136160283048888");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.7712130899609478");
tankValues.add("-0.95!-0.7712130899609478");
tankValues.add("-0.95!-0.6880760788685868");
tankValues.add("-0.95!-0.8331634131192878");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.18128921853327953");
tankValues.add("-0.95!0.02031613993875592");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.1587278219023179");
tankValues.add("-0.95!-0.013405156939550725");
tankValues.add("-0.95!-0.5766798563499881");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.8081281053467821!-0.95");
tankValues.add("-0.34572183395333417!-0.95");
tankValues.add("-0.2214728019475803!-0.95");
tankValues.add("-0.14471987699366906!-0.95");
tankValues.add("-0.036706281462876404!-0.95");
tankValues.add("-0.036706281462876404!-0.95");
tankValues.add("0.005152919723156903!-0.95");
tankValues.add("0.09822486647915947!-0.95");
tankValues.add("0.09822486647915947!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.95!-0.95");
tankValues.add("0.9232406900987306!-0.95");
tankValues.add("0.0839850500238489!-0.95");
tankValues.add("-0.0!-0.95");
tankValues.add("-0.02249891778413872!-0.95");
tankValues.add("-0.04667946769919533!-0.95");
tankValues.add("-0.05770327138459933!-0.95");
tankValues.add("-0.05770327138459933!-0.95");
tankValues.add("-0.28835317911243985!-0.95");
tankValues.add("-0.30427179937721155!-0.95");
tankValues.add("-0.25022999252553135!-0.95");
tankValues.add("-0.2805367115461971!-0.95");
tankValues.add("-0.45467358742188885!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.8331634131192878");
tankValues.add("-0.95!-0.05023861701945412");
tankValues.add("-0.95!-0.0");
tankValues.add("-0.95!0.02820768563610347");
tankValues.add("-0.95!0.11344704987401207");
tankValues.add("-0.95!0.12963599630516248");
tankValues.add("-0.95!0.025449833264210967");
tankValues.add("-0.95!0.0028120491842688637");
tankValues.add("-0.95!-0.033620150414736844");
tankValues.add("-0.95!-0.033620150414736844");
tankValues.add("-0.95!-0.5982586824610991");
tankValues.add("-0.95!-0.7590787927113272");
tankValues.add("-0.95!-0.845808222491916");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.036706281462876404");
tankValues.add("-0.95!0.17109205440058256");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!0.95");
tankValues.add("-0.95!-0.18773778018981926");
tankValues.add("-0.95!-0.18773778018981926");
tankValues.add("-0.95!-0.7590787927113272");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.95!-0.95");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.7!-0.7");
tankValues.add("-0.0!-0.003743661444161224");
    
        return tankValues;
    }

    public String evaluatePath() {
    if (Constants.kBackLL.getTy() < 0) {
      return "ARED";
    } else {
      return "BBLUE";
        }
    }
}
