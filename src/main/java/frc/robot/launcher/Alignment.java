package frc.robot.launcher;

import frc.robot.Constants;
import frc.robot.limelight.Distance;
import frc.robot.limelight.LimeLight;

class Alignment {
    private Distance distance;
    private LimeLight limeLight;
    private int pipeline;

    private double velocity;
    private double maxDisplacement;

    Alignment() {
        this.distance = new Distance(Constants.MOUNTING_HEIGHT, Constants.MOUNTING_ANGLE, Constants.REFRENCE_HEIGHT);
        this.pipeline = Constants.PORT_PIPELINE;
        this.velocity = Constants.SHOOTER_MAX_VELOCITY;

        limeLight = new LimeLight();

        maxDisplacement = getDistance(Math.atan(Math.pow(velocity, 2) / 9.81));
    }

    private double getTime(double theta) {
        return Math.sqrt(2 * distance.getHeight() / 9.81 + Math.pow(velocity * Math.sin(theta) / 9.81, 2))
                + velocity * Math.sin(theta) / 9.81;
    }

    private double getErrorAngle() {
        limeLight.selectPipeline(pipeline);
        if (limeLight.getTv() == 0) {
            return -1;
        }

        return distance.getAngle(limeLight.getTy());
    }

    private double getDistance(double theta) {
        return velocity * Math.cos(theta) * getTime(theta);
    }

    public double getAngle() {
        double theta = getErrorAngle();
        if (theta < 0) {
            return -1;
        }
        if (theta > 90) {
            return -1;
        }

        if (getDistance(theta) > maxDisplacement) {
            return -1;
        }

        return Math.acos(getDistance(theta) / (velocity * getTime(theta)));
    }
}