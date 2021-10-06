package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private String table;

    public LimeLight() {
        table = "limelight";
    }

    private double GetEntry(String selector) {
        return NetworkTableInstance.getDefault().getTable(table).getEntry(selector).getDouble(0);
    }

    public double getTv() {
        return GetEntry("tv");
    }

    public double getTx() {
        return GetEntry("tx");
    }

    public double getTy() {
        return GetEntry("ty");
    }

    public double getTa() {
        return GetEntry("ta");
    }

    public double getTs() {
        return GetEntry("ts");
    }

    public double getTl() {
        return GetEntry("tl");
    }

    public double getTshort() {
        return GetEntry("tshort");
    }

    public double getTlong() {
        return GetEntry("tlong");
    }

    public double getThor() {
        return GetEntry("thor");
    }

    public double getTvert() {
        return GetEntry("tvert");
    }

    public double getPipeline() {
        return GetEntry("getpipe");
    }

    public double getCamtran() {
        return GetEntry("camtran");
    }

    private void setEntry(String selector, double value) {
        NetworkTableInstance.getDefault().getTable(table).getEntry(selector).setNumber(value);
    }

    private void pipelineLight() {
        setEntry("ledMode", 0);
    }

    public void disableLight() {
        setEntry("ledMode", 1);
    }

    public void blinkLight() {
        setEntry("ledMode", 2);
    }

    public void enableLight() {
        setEntry("ledMode", 3);
    }

    public void enableVisionMode() {
        setEntry("camMode", 0);
    }

    public void enableDriveMode() {
        setEntry("camMode", 1);
    }

    public void selectPipeline(int selector) {
        if (selector >= 0 && selector <= 9) {
            setEntry("pipeline", selector);
            pipelineLight();
            return;
        }

        throw new IllegalArgumentException("pipline must be selected from 0 to 9");
    }
}