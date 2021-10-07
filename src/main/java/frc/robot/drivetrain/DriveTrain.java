
package frc.robot.drivetrain;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PIDControl;
import frc.robot.Robot;
 
/**
 * Modified basic tank drive drivetrain for recording
 */
public class DriveTrain {
    private CANSparkMax laMotor, lbMotor, lcMotor, raMotor, rbMotor, rcMotor;
    private SpeedControllerGroup lGroup, rGroup;

    private DifferentialDrive drive;

    private boolean invert;

    private PIDControl pidControl;
    private Alignment alignment;

    private ADXRS450_Gyro mGyro;

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

        laMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        lbMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        lcMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        raMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        rbMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        rcMotor.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);

        laMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        lbMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        lcMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        raMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        rbMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        rcMotor.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);

        lGroup = new SpeedControllerGroup(laMotor, lbMotor, lcMotor);
        rGroup = new SpeedControllerGroup(raMotor, rbMotor, rcMotor);

        drive = new DifferentialDrive(lGroup, rGroup);

        // Modified
        i = j = 0;

        toggle = active = record = false;
        invert =true;
        jsonObj = new JSONObject();

        pidControl = new PIDControl(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KP);
        pidControl.setMaxSpeed(Constants.DRIVE_MAX_ROTATION_SPEED);
        pidControl.setTolerance(Constants.DRIVE_ROTATION_TOLERANCE);
        alignment = new Alignment();

        
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

        if (Constants.driveController.getBButton()) {
            align();
        } else if (Constants.driveController.getAButton()) {
            forward();
        } else if (Constants.driveController.getYButton()) {
            backward();
        } else {
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

    public void align() {
        if (alignment.targetFound()) {
            drive.arcadeDrive(0, pidControl.getValue(0, alignment.getError()));
        }
    }

    public ArrayList<String> threeTrench() {
    
    ArrayList<String> tankValues = new ArrayList<String>();
    tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-0.0!-9.664364580856157E-5");
tankValues.add("-0.0!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-0.0!-0.0");
tankValues.add("-0.0!-9.664364580856157E-5");
tankValues.add("-0.0!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-9.664364580856157E-5");
tankValues.add("-9.664364580856157E-5!-0.0");
tankValues.add("-9.664364580856157E-5!-3.3653272119053753E-4");
tankValues.add("-9.664364580856157E-5!-6.982188405351056E-4");
tankValues.add("-9.664364580856157E-5!-6.982188405351056E-4");
tankValues.add("-3.3653272119053753E-4!-0.0011718749999999997");
tankValues.add("-0.007239026030672056!-0.011173873985158409");
tankValues.add("-0.16748900170817693!-0.16273401745777144");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.3376312311712875");
tankValues.add("-0.6!-0.04673307453895021");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-3.3653272119053753E-4");
tankValues.add("-0.6!-6.982188405351056E-4");
tankValues.add("-0.6!-6.982188405351056E-4");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0");
tankValues.add("-0.6!-0.0032088526664239064");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.41288900285358593!-0.6");
tankValues.add("-0.12690597576730628!-0.6");
tankValues.add("-0.03890968916791824!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.6");
tankValues.add("-0.0!-0.4");
tankValues.add("-0.0074492493234389395!-0.4");
tankValues.add("-0.0074492493234389395!-0.4");
tankValues.add("-0.07633230254032823!-0.4");
tankValues.add("-0.20796203470203323!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.3943925873018052");
tankValues.add("-0.4!-0.28486074347074375");
tankValues.add("-0.4!-0.0874449808644096");
tankValues.add("-0.4!-0.006518986476616033");
tankValues.add("-0.4!-0.0");
tankValues.add("-0.4!0.0011840207091658374");
tankValues.add("-0.4!0.004122999581274385");
tankValues.add("-0.4!0.004894633266876724");
tankValues.add("-0.4!0.004894633266876724");
tankValues.add("-0.4!4.7209733538517936E-4");
tankValues.add("-0.4!-4.654792270234038E-4");
tankValues.add("-0.4!-0.036791212149114814");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.6!-0.6");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.4!-0.4");
tankValues.add("-0.01420984281103498!-0.6");
tankValues.add("0.021535677647688597!-0.6");
tankValues.add("0.12445178670031092!-0.4567516640935834");
tankValues.add("-0.0!-0.0");
tankValues.add("9.664364580856157E-5!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.0");
tankValues.add("0.0!0.005044403542514114");
tankValues.add("6.982188405351056E-4!0.20770087814772284");
tankValues.add("0.021233779209307482!0.6");
tankValues.add("0.3441941157686658!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.24600480498528146!0.6");
tankValues.add("0.019361089239960744!0.6");
tankValues.add("3.3653272119053753E-4!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("0.0!0.6");
tankValues.add("6.982188405351056E-4!0.6");
tankValues.add("0.04144667553473762!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.449304741082277");
tankValues.add("0.6!0.19728845343574902");
tankValues.add("0.6!-3.413174889885862E-4");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.6!9.664364580856157E-5");
tankValues.add("0.4!6.442909720570772E-5");
tankValues.add("0.4!6.442909720570772E-5");
tankValues.add("0.4!6.442909720570772E-5");
tankValues.add("0.4!2.2435514746035838E-4");
tankValues.add("0.4!0.0033629356950094095");
tankValues.add("0.4!0.0492935425375151");
tankValues.add("0.4!0.164003203323521");
tankValues.add("0.4!0.31453898917111395");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.28004175087918315");
tankValues.add("0.4!0.0874449808644096");
tankValues.add("0.4!0.012907392826640496");
tankValues.add("0.4!2.2435514746035838E-4");
tankValues.add("0.4!4.654792270234038E-4");
tankValues.add("0.4!0.00116742248509526");
tankValues.add("0.4!0.0021392351109492712");
tankValues.add("0.4!0.0074492493234389395");
tankValues.add("0.6!0.1443257069340701");
tankValues.add("0.6!0.2237640946294541");
tankValues.add("0.6!0.39870677384503744");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.6!0.6");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.4!0.4");
tankValues.add("0.32986645908043555!0.1349766104823344");
tankValues.add("0.31961212324687466!0.07633230254032823");
tankValues.add("-0.039312959487298314!0.0021392351109492712");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("6.442909720570772E-5!2.2435514746035838E-4");
tankValues.add("6.442909720570772E-5!2.2435514746035838E-4");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!6.442909720570772E-5");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("0.0!6.442909720570772E-5");
tankValues.add("0.0!2.2435514746035838E-4");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!2.2435514746035838E-4");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");
tankValues.add("6.442909720570772E-5!6.442909720570772E-5");

    return tankValues;
    }

}