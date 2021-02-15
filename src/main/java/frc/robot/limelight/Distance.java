package frc.robot.limelight;


public class Distance {
    private double mountingHeight, refrenceHeight; // Height relative to the ground in meters
    private double mountingAngle; // Angle relative to ground plane in degrees

    public Distance(double mountingHeight, double mountingAngle, double refrenceHeight) {
        this.mountingHeight = mountingHeight;
        this.mountingAngle = mountingAngle;
        this.refrenceHeight = refrenceHeight;
    }

    public double getDistance(double ty) {
        return (getHeight()) / Math.tan(getAngle(Math.toRadians(ty)));
    }

    public double getHeight() {
        return refrenceHeight - mountingHeight;
    }

    public double getAngle(double ty) {
        return ty + mountingAngle;
    }
}