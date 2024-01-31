#include "PIDController.h"
#include <string>
#include <climits>
#include <cmath>


namespace rms_utils {

// #ifdef ENABLE_PID_CONTROLLER

// PID controller courtesy of Peter Tischler, with modifications.


/**
 * Allocate a PID object with the given constants for P, I, D
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 */
PIDController::PIDController(double Kp, double Ki, double Kd)
{
    m_P = Kp;
    m_I = Ki;
    m_D = Kd;
}

/**
 * Read the input, calculate the output accordingly, and write to the output.
 * This should only be called by the PIDTask
 * and is created during initialization.
 */
void PIDController::calculate()
{
    int     sign = 1;

    // If enabled then proceed into controller calculations
    if (m_enabled)
    {
        // Calculate the error signal
        m_error = m_setpoint - m_input;

        // If continuous is set to true allow wrap around
        if (m_continuous)
        {
            if (std::fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
            {
                if (m_error > 0)
                    m_error = m_error - m_maximumInput + m_minimumInput;
                else
                    m_error = m_error + m_maximumInput - m_minimumInput;
            }
        }

        // Integrate the errors as long as the upcoming integrator does
        // not exceed the minimum and maximum output thresholds.

        if ((std::fabs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                (std::fabs(m_totalError + m_error) * m_I > m_minimumOutput))
            m_totalError += m_error;

        // Perform the PID calculation
        m_result = (m_P * m_error) + (m_I * m_totalError) + (m_D * (m_error - m_prevError));

        // Save the current error to the previous error for the next cycle.
        m_prevError = m_error;

        if (m_result < 0) sign = -1;    // Record sign of result.

        // Make sure the final result is within bounds. If we constrain the result, we make
        // sure the sign of the constrained result matches the original result sign.
        if (std::fabs(m_result) > m_maximumOutput)
            m_result = m_maximumOutput * sign;
        else if (std::fabs(m_result) < m_minimumOutput)
            m_result = m_minimumOutput * sign;
    }
}

/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void PIDController::setPID(double p, double i, double d)
{
    m_P = p;
    m_I = i;
    m_D = d;
}

/**
 * Get the Proportional coefficient
 * @return proportional coefficient
 */
double PIDController::getP()
{
    return m_P;
}

/**
 * Get the Integral coefficient
 * @return integral coefficient
 */
double PIDController::getI()
{
    return m_I;
}

/**
 * Get the Differential coefficient
 * @return differential coefficient
 */
double PIDController::getD()
{
    return m_D;
}

/**
 * Return the current PID result for the last input set with setInput().
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
double PIDController::performPID()
{
    calculate();
    return m_result;
}

/**
 * Return the current PID result for the specified input.
 * @param input The input value to be used to calculate the PID result.
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
double PIDController::performPID(double input)
{
    setInput(input);
    return performPID();
}

/**
 *  Set the PID controller to consider the input to be continuous,
 *  Rather then using the max and min in as constraints, it considers them to
 *  be the same point and automatically calculates the shortest route to
 *  the setpoint.
 * @param continuous Set to true turns on continuous, false turns off continuous
 */
void PIDController::setContinuous(bool continuous)
{
    m_continuous = continuous;
}

/**
 *  Set the PID controller to consider the input to be continuous,
 *  Rather then using the max and min in as constraints, it considers them to
 *  be the same point and automatically calculates the shortest route to
 *  the setpoint.
 */
void PIDController::setContinuous()
{
    this->setContinuous(true);
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input, always positive
 * @param maximumInput the maximum value expected from the output, always positive
 */
void PIDController::setInputRange(double minimumInput, double maximumInput)
{
    m_minimumInput = std::fabs(minimumInput);
    m_maximumInput = std::fabs(maximumInput);
    setSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output, always positive
 * @param maximumOutput the maximum value to write to the output, always positive
 */
void PIDController::setOutputRange(double minimumOutput, double maximumOutput)
{
    m_minimumOutput = std::fabs(minimumOutput);
    m_maximumOutput = std::fabs(maximumOutput);
}

/**
 * Set the setpoint for the PIDController
 * @param setpoint the desired setpoint
 */
void PIDController::setSetpoint(double setpoint)
{
    int     sign = 1;

    if (m_maximumInput > m_minimumInput)
    {
        if (setpoint < 0) sign = -1;

        if (std::fabs(setpoint) > m_maximumInput)
            m_setpoint = m_maximumInput * sign;
        else if (std::fabs(setpoint) < m_minimumInput)
            m_setpoint = m_minimumInput * sign;
        else
            m_setpoint = setpoint;
    }
    else
        m_setpoint = setpoint;
}

/**
 * Returns the current setpoint of the PIDController
 * @return the current setpoint
 */
double PIDController::getSetpoint()
{
    return m_setpoint;
}

/**
 * Retruns the current difference of the input from the setpoint
 * @return the current error
 */
double PIDController::getError()
{
    return m_error;
}

/**
 * Set the percentage error which is considered tolerable for use with
 * OnTarget. (Input of 15.0 = 15 percent)
 * @param percent error which is tolerable
 */
void PIDController::setTolerance(double percent)
{
    m_tolerance = percent;
}

/**
 * Return true if the error is within the percentage of the total input range,
 * determined by setTolerance. This assumes that the maximum and minimum input
 * were set using setInputRange.
 * @return true if the error is less than the tolerance
 */
bool PIDController::onTarget()
{
    return (std::fabs(m_error) < std::fabs(m_tolerance / 100.0 * (m_maximumInput - m_minimumInput)));
}

/**
 * Begin running the PIDController
 */
void PIDController::enable()
{
    m_enabled = true;
}

/**
 * Stop running the PIDController.
 */
void PIDController::disable()
{
    m_enabled = false;
}

/**
 * Reset the previous error,, the integral term, and disable the controller.
 */
void PIDController::reset()
{
    disable();
    m_prevError = 0;
    m_totalError = 0;
    m_result = 0;
}

/**
 * Set the input value to be used by the next call to performPID().
 * @param input Input value to the PID calculation.
 */
void PIDController::setInput(double input)
{
    int     sign = 1;

    if (m_maximumInput > m_minimumInput)
    {
        if (input < 0) sign = -1;

        if (std::fabs(input) > m_maximumInput)
            m_input = m_maximumInput * sign;
        else if (std::fabs(input) < m_minimumInput)
            m_input = m_minimumInput * sign;
        else
            m_input = input;
    }
    else
        m_input = input;
}

// #endif

}// namespace rms_utils