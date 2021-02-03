package frc.robot.launcher;

import java.util.ArrayList;
import java.util.Collections;

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

    public double get_theta(){
        //creates the array lists for storing values 
        double ty = limeLight.getTy();
        double dist = distance.getDistance(ty);
        
        ArrayList<Double> theta_list = new ArrayList<>();       
        ArrayList<Double> difference_list =  new ArrayList<>();

        //adds all the desired values to test for theta to theta_list
        double number_to_add = 0;
        while (number_to_add <= 90){
            theta_list.add(number_to_add);
            number_to_add += 0.1;           //this is the increment between each theta value tested
        }
        
        //calculates the difference between the left and right side of the equation for each theta value and stores it in difference_list
        for (double theta : theta_list){
            double theta_input = Math.toRadians(theta);
            double leftside = dist + Constants.LLPIV - Constants.LT*Math.cos(theta_input);
            double rightside = Constants.V*Math.cos(theta_input)*(Math.sqrt((2*(Constants.HP-Constants.LT*Math.sin(theta_input)-Constants.HPIV)/Constants.G)));
            double difference = Math.abs(leftside - rightside);
            difference_list.add(difference);
        }

        //finds the index of the minimum difference
        int index_of_min = difference_list.indexOf(Collections.min(difference_list));

        //gets and returns the best theta value for the smallest difference between the right and left side of the equation
        double theta_value = theta_list.get(index_of_min);
        return theta_value;
    }

}