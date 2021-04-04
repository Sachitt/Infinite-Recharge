package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private NetworkTable mNT;

    public LimeLight(String name) {
        mNT = NetworkTableInstance.getDefault().getTable(name);
    }

    public boolean getTv() {
        return mNT.getEntry("tv").getDouble(0) == 1 ? true : false;
    }

    public double getTx() {
        return mNT.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return mNT.getEntry("ty").getDouble(0);
    }

    public double getTa() {
        return mNT.getEntry("ta").getDouble(0);
    }

    public double getTs() {
        return mNT.getEntry("ts").getDouble(0);
    }

    public double getTl() {
        // add 11 for image capture latency
        return mNT.getEntry("tl").getDouble(0) + 11;
    }

    public double getTshort() {
        return mNT.getEntry("tshort").getDouble(0);
    }

    public double getTlong() {
        return mNT.getEntry("tlong").getDouble(0);
    }

    public double getThor() {
        return mNT.getEntry("thor").getDouble(0);
    }

    public double getPipeline() {
        return mNT.getEntry("getpipe").getDouble(0);
    }

    public void setLedPipe() {
        mNT.getEntry("ledMode").setNumber(0);
    }

    public void setLedOff() {
        mNT.getEntry("ledMode").setNumber(1);
    }

    public void setLedBlink() {
        mNT.getEntry("ledMode").setNumber(2);
    }

    public void setLedOn() {
        mNT.getEntry("ledMode").setNumber(3);
    }

    public void setVision() {
        mNT.getEntry("camMode").setNumber(0);
    }

    public void setDriver() {
        mNT.getEntry("camMode").setNumber(1);
    }

    public void setStandard() {
        mNT.getEntry("stream").setNumber(0);
    }

    public void setPrimary() {
        mNT.getEntry("stream").setNumber(1);
    }

    public void setSecondary() {
        mNT.getEntry("stream").setNumber(2);
    }

    public void setPipeline(int pipeline) {
        setPipeline(pipeline, true);
    }

    public void setPipeline(int pipeline, Boolean ledFromPipe) {
        mNT.getEntry("pipeline").setNumber(pipeline);
        if (ledFromPipe) {
            setLedPipe();
        }
    }
}