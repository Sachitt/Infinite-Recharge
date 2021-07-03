package frc.robot;

import edu.wpi.first.networktables.*;

public class Limelight {
    public NetworkTable table;
    public double Distance;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double returnEntry(String key) {
        return table.getEntry(key).getDouble(0);
    }

    public double getTv() {
        return returnEntry("tv");
    }

    public double getTx() {
        return returnEntry("tx");
    }

    public double getTy() {
        return returnEntry("ty");
    }

    public double getTa() {
        return returnEntry("ta");
    }

    public double getTs() {
        return returnEntry("ts");
    }

    public double getTl() {
        return returnEntry("tl");
    }

    private void setEntry(String key, double value) {
        table.getEntry(key).setNumber(value);
    }

    public void setLEDMode(double value) {
        setEntry("ledMode", value);
    }

    public void selectPipeline(int number) {
        if (number >= 0 && number <= 9) {
            setEntry("pipline", number);
            setLEDMode(0);
            return;
        }
        throw new IllegalArgumentException("Values range from 0-9");
    }

    public void setMode(double value) {
        setEntry("camMode", value);
    }

    public boolean targetFound() {
        selectPipeline(Constants.PORT_PIPELINE);
        if (getTy() == 0) {
            return false;
        }
        return true;
    }

    public double getXError() {
        if (targetFound()) {
            return getTx();
        }
        return 0;
    }

    public void getDistance(double mountingHeight, double mountingAngle, double referenceHeight) {
        double Height = referenceHeight - mountingHeight;
        double Angle = Math.toRadians(getTy()) + mountingAngle;

        Distance = (Height) / Math.tan(Angle);
    }

}
