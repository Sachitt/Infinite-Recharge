package com.team2568.frc2020.fsm.teleop;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.team2568.frc2020.Constants;
import com.team2568.frc2020.Registers;
import com.team2568.frc2020.fsm.FSM;
import com.team2568.frc2020.fsm.auto.DriveTrain.DriveAutoMode;
import com.team2568.frc2020.fsm.auto.DriveTrain.TankValue;
import com.team2568.frc2020.states.DriveState;
import com.team2568.frc2020.subsystems.DriveTrain.DriveMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A button inverts controls. Left and right Y axis controls the left and right
 * speed respectively. LB and RB changes the drive power that is used in the
 * curve function.
 * 
 * @author Ryan Chaiyakul
 */
public class DriveTrain extends FSM {
    private static DriveTrain mInstance;

    private DriveState nState;

    private DriveMode nMode;

    private double left, right;
    private double forward, rotation;

    private int constL, constR;
    private double drivePower;

    private boolean isInverted;

    private JoystickMap mMap;

    private Integer i;
    @JsonSerialize(using = TankValueSeralizer.class)
    private ArrayList<TankValue> mRecording;

    class TankValueSeralizer extends JsonSerializer<List<TankValue>> {

        @Override
        public void serialize(List<TankValue> value, JsonGenerator jgen, SerializerProvider provider)
                throws IOException {
            jgen.writeStartArray();
            for (TankValue model : value) {
                jgen.writeStartObject();
                jgen.writeObjectField("TankValue", model);
                jgen.writeEndObject();
            }
            jgen.writeEndArray();
        }

    }

    private class JoystickMap {
        private List<TankValue> mUp, mRight, mDown, mLeft;

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

    public static DriveTrain getInstance() {
        if (mInstance == null) {
            mInstance = new DriveTrain();
        }
        return mInstance;
    }

    private DriveTrain() {
        isInverted = false;
    };

    public void compute() {
        nState = Registers.kDriveState.get();

        switch (Registers.kDriveState.get()) {
        case STANDARD:
            nMode = DriveMode.kTank;
            if (isInverted) {
                left = -Constants.kDriveController.getRightY();
                right = -Constants.kDriveController.getLeftY();
            } else {
                left = Constants.kDriveController.getLeftY();
                right = Constants.kDriveController.getRightY();
            }

            forward = 0;
            rotation = 0;

            // X will invert the controls
            if (Constants.kDriveController.getXButtonPressed()) {
                isInverted = !isInverted;
            } else if (Constants.kDriveController.getAButton()) {
                nState = DriveState.FORWARD;
            } else if (Constants.kDriveController.getYButton()) {
                nState = DriveState.REVERSE;
            } else if (Constants.kDriveController.getBButton()) {
                nState = DriveState.TARGET;
                Registers.kDriveAutoMode.set(DriveAutoMode.kTarget);
            } else if (Constants.kDriveController.getPOV() != -1) {
                i = Constants.kDriveController.getPOV();

                if (mMap.get(i).size() == 0) {
                    nState = DriveState.RECORD;
                    mRecording = new ArrayList<TankValue>();
                } else {
                    nState = DriveState.PLAYBACK;
                    Registers.kDriveAutoMode.set(DriveAutoMode.kFollow);
                    Registers.kDriveAutoTankList.set(mMap.get(i));
                }
            }
            break;
        case FORWARD:
            nMode = DriveMode.kArcade;
            left = 0;
            right = 0;
            // Drive power is conicidently the same as the constant speed we use for forward
            // and backwards
            forward = getDrivePower();
            rotation = 0;

            if (!Constants.kDriveController.getAButton()) {
                nState = DriveState.STANDARD;
            }
            break;
        case REVERSE:
            nMode = DriveMode.kArcade;
            left = 0;
            right = 0;
            // Drive power is conicidently the same as the constant speed we use for forward
            // and backwards
            forward = -getDrivePower();
            rotation = 0;

            if (!Constants.kDriveController.getAButton()) {
                nState = DriveState.STANDARD;
            }
            break;
        case TARGET:
            nMode = DriveMode.kArcade;
            left = 0;
            right = 0;
            forward = 0;
            rotation = Registers.kDriveAutoDriveZ.get();

            if (!Constants.kDriveController.getBButton()) {
                Registers.kDriveAutoMode.set(DriveAutoMode.kOff);
                nState = DriveState.STANDARD;
            }
            break;
        case RECORD:
            nMode = DriveMode.kTank;
            if (isInverted) {
                left = -Constants.kDriveController.getRightY();
                right = -Constants.kDriveController.getLeftY();
            } else {
                left = Constants.kDriveController.getLeftY();
                right = Constants.kDriveController.getRightY();
            }

            forward = 0;
            rotation = 0;

            mRecording.add(new TankValue(left, right));

            if (!Constants.kDriveController.getAButton()) {
                nState = DriveState.STANDARD;
                mMap.set(i, mRecording);

                try {
                    String serialized = new ObjectMapper().writeValueAsString(mRecording);
                    SmartDashboard.putString("DriveRecording", serialized);
                } catch (IOException ex) {
                    return;
                }
            }
            break;
        case PLAYBACK:
            nMode = DriveMode.kTank;
            left = Registers.kDriveAutoL.get();
            right = Registers.kDriveAutoR.get();
            forward = 0;
            rotation = 0;

            if (!Constants.kDriveController.getBButton()) {
                Registers.kDriveAutoMode.set(DriveAutoMode.kOff);
                nState = DriveState.STANDARD;
            }
            break;
        case STOP:
            nMode = DriveMode.kOff;
            left = 0;
            right = 0;
            forward = 0;
            rotation = 0;
            break;
        }

        setDirection();

        Registers.kDriveL.set(constL * toDriveSpeed(left));
        Registers.kDriveR.set(constR * toDriveSpeed(right));
        Registers.kDriveF.set(forward);
        Registers.kDriveZ.set(rotation);

        Registers.kDriveMode.set(nMode);
        Registers.kDriveState.set(nState);
    }

    private double toDriveSpeed(double joystick) {
        // However driveExponent should be constant (Changeable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("Drive Exponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        return getDrivePower() * Math.pow(Math.abs(joystick), driveExponent);
    }

    private double getDrivePower() {
        // LB/RB can change the drivePower during the match
        drivePower = Constants.kDriveRegularPower;
        if (Constants.kDriveController.getLeftBumper()) {
            drivePower = Constants.kDriveSlowPower;
        } else if (Constants.kDriveController.getRightBumper()) {
            drivePower = Constants.kDriveTurboPower;
        }

        return drivePower;
    }

    private void setDirection() {
        // Easily inverted if wired strangely
        constL = 1;
        constR = 1;

        // Use a constant multiplier for +/- direction as the driveExponent could be
        // even and negate the sign
        if (right < 0) {
            constR *= 1;
        } else if (right > 0) {
            constR *= -1;
        }

        if (left < 0) {
            constL *= 1;
        } else if (left > 0) {
            constL *= -1;
        }
    }

    public void writeDashboard() {
        SmartDashboard.putString("DriveState", nState.toString());
    }
}
