package frc.robot;
// TODO: add sd support and inRange indicator

/**
 * Custom PID controller class to make pid work! Configurable items: p, i, d,
 * scale, tolerance This class isn't for the most accurate pid in the world. Its
 * to get the job done
 * 
 * <p>
 * Bugs: None yet!
 *
 * @author Neal Chokshi
 */
public class PIDControl {
    private float kP = 0, kI = 0, kD = 0; // tuning values
    private double accumulator = 0, error = 0, scale = 1.0, output = 0, lastError = 0, prevTime = 0; // modifiers for
                                                                                                     // pid
    private double tolerance = 1, maxSpeed = 1;;
    private boolean inRange = false;

    /**
     * Constructor for PIDControl. Use this if your encode does not implement
     * PIDSource. (im watching you CANEncoder...)
     *
     * @param p proportional tuning value
     * @param i integral tuning value
     * @param d differential tuning value
     */
    public PIDControl(float p, float i, float d) {
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * Determines how accurate pid has to be for it to return true for in range
     * value
     * 
     * @param tolerance the tolerance to set 0 <= tolerance <= 1. 0 = full
     *                  tolerance, 1 - no tolerance
     * 
     */
    public void setTolerance(double tolerance) {
        if (tolerance > 0) {
            this.tolerance = 1;
        } else {
            this.tolerance = tolerance;
        }
    }

    public boolean isInRange() {
        return inRange;
    }

    /**
     * Enforce a max speed for pid
     * 
     * @param max a speed between 0 and 1
     * 
     */
    public void setMaxSpeed(double max) {
        maxSpeed = max;
    }

    /**
     * Set a value for the proportional tuning value
     *
     * @param p value
     */
    public void setP(float p) {
        kP = p;
    }

    /**
     * Set a value for the integral tuning value
     *
     * @param i value
     */
    public void setI(float i) {
        kI = i;
    }

    /**
     * Update all tuning values simultanouesly.
     *
     * @param p value
     * @param i value
     * @param d value
     */
    public void setPID(float p, float i, float d) {
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * Set a value for the differential tuning value
     *
     * @param d value
     */
    public void setD(float d) {
        kD = d;
    }

    /**
     * Set a constant to scale input by a number
     *
     * @param scale value
     */
    public void setScale(double scale) {
        this.scale = scale;
    }

    /**
     * Get value returned by pid calculations. Use this method if the pidGet
     * function is already taken or you need to use a custom value for input.
     *
     * @param setpoint where the pid input value should be
     * @param current  value of the sensor
     * @return double speed value
     */
    public double getValue(double setpoint, double currentValue) {
        error = setpoint - currentValue;
        accumulator += error;

        double slope = 0;
        if (prevTime != 0) {
            slope = 1000 * ((error - lastError) / (System.currentTimeMillis() - prevTime));
        }
        prevTime = System.currentTimeMillis();
        lastError = error;

        output = (error * kP + accumulator * kI + slope * kD) * scale;

        if (Math.abs(output) < 0.02 && Math.abs(error) < (setpoint - setpoint * tolerance)) {
            inRange = true;
        } else {
            inRange = false;
        }

        if (Math.abs(output) > maxSpeed) {
            return (output > 0 ? 1 : -1) * maxSpeed;
        } else {
            return output;
        }
    }

    /**
     * Reset pid loop and all values used
     * 
     * @return void
     */
    public void cleanup() {
        accumulator = 0;
        output = 0;
        prevTime = 0;
        lastError = 0;
        inRange = false;
    }

}