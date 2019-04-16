package org.firstinspires.ftc.teamcode;

public class PID {

    /**
     * Gains For PID, these are the values that control how the pid operates
     * They are shorted to what they are commonly referred to.
     */
    // Proportional Gain
    private double kp;
    // Setter Method For Proportional (Should only be used during testing)
    public void setKp (double k) {
        kp = k;
    }
    // Integral Gain
    private double ki;

    //Derivative Gain
    private long kd;
    // Setter for the Derivative Gain (Should only be used for testing)
    public void setKd (long k) {
        kd =  k;
    }

    //Inegral Error which is continuously added when called
    private double errori = 0;

    //Derivative error wich tores the previous error
    private double errord = 0;

    /**
     * Constructor for creating a PID object. The
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    public PID(double kp, double ki, long kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Takes the inputted error to calculate the proportional value to
     * change the
     * @param error The error of the system
     * @return The proportional Part of the Derivative
     */
    private double proportional(double error) {
        return kp * error;
    }

    /**
     * Calculates the integral part of the pid loop
     * which is the gain times the total error of the system
     * @param error
     * @return
     */
    private double integral(double error) {
        errori += error;
        return ki * errori;
    }

    /**
     * Calcuates the derivative part of the pid loop
     * which is the change in error between the last two iterations
     * @param error
     * @return
     */
    private double derivative(double error) {
        double temp = errord;
        errord = error;
        return kd * (error - errord);
    }

    /**
     * Calcuates the total pid value of the system
     * @param error
     * @return the pid value
     */
    public double update(double error) {
        return proportional(error) + integral(error) + derivative(error);
    }


}
