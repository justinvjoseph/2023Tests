#pragma once

namespace rms_utils {
// PID controller courtesy of Peter Tischler, with modifications.
// #ifdef ENABLE_PID_CONTROLLER


class PIDController
{
private:
    double m_P;                     // factor for "proportional" control
    double m_I;                     // factor for "integral" control
    double m_D;                     // factor for "derivative" control
    double m_input;                 // sensor input for pid controller
    double m_maximumOutput = 1.0;	// |maximum output|
    double m_minimumOutput = 0.0;	// |minimum output|
    double m_minimumInput = 0.0;	// minimum input - limit setpoint to this
    double m_maximumInput = 0.0;	// maximum input - limit setpoint to this
    bool m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    bool m_enabled = false;      // is the pid controller enabled
    double m_prevError = 0.0;       // the prior sensor input (used to compute velocity)
    double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    double m_tolerance = 1.0;      // the percentage error that is considered on target
    double m_setpoint = 0.0;
    double m_error = 0.0;
    double m_result = 0.0;

public:
    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    PIDController(double Kp, double Ki, double Kd);

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    void setPID(double p, double i, double d);

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    double getP();

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    double getI();

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    double getD();

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    double performPID();

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    double performPID(double input);

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    void setContinuous(bool continuous);

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    void setContinuous();

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    void setInputRange(double minimumInput, double maximumInput);    

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    void setOutputRange(double minimumOutput, double maximumOutput);

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    void setSetpoint(double setpoint);

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    double getSetpoint();

    /**
     * Retruns the current difference of the input from the setpoint
     * @return the current error
     */
    double getError();

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */
    void setTolerance(double percent);

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * @return true if the error is less than the tolerance
     */
    bool onTarget();

    /**
     * Begin running the PIDController
     */
    void enable();

    /**
     * Stop running the PIDController.
     */
    void disable();

    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    void reset();

    /**
     * Set the input value to be used by the next call to performPID().
     * @param input Input value to the PID calculation.
     */
    void setInput(double input);

private:
    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    void calculate();
};

// #endif

} // namespace rms_utils