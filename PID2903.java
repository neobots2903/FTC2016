public class PID2903
{
	public enum OPERATING_MODE
	{
		MANUAL,
		AUTOMATIC
	}

	public enum TUNING_DIRECTION
	{
		DIRECT,
		REVERSE
	}

    // private data
    private long lastTime;
    private double input, output, setpoint;
    private double iTerm, lastInput;
    private double kp, ki, kd;
    private long sampleTime;
    private double outMin, outMax;
    private boolean inAuto;

    private TUNING_DIRECTION controllerDirection;

    /*
     * PID2903 constructor
     * 
     * Specifies default values for proportional, integral and derivative
     * values as well as the direction tuning should go, forward or
     * backward
     * 
     * Parameters
     * 	kp			proportional value
     * 	ki			integral value
     * 	kd			derivative value
     * 	direction	direction for tuning (forward/reverse)
     * 
     * Returns
     * 	newly instantiated object.
     */
	public PID2903(	double kp, double ki, double kd,
					TUNING_DIRECTION direction)
	{
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.controllerDirection = direction;
		
		init();
	}
	
    /*
     * compute
     * 
     * This public method computes the output value.  This should be called 
     * from a loop method which calls this repeatedly until the desired output
     * value is reached. 
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	generated output.  If not enough time has elapsed, then a new output
     *  value is not generated.
     */
    public double compute()
    {
        if (inAuto)
        {
	        long now = System.currentTimeMillis();
	        long  timeChange = (now - lastTime);
	        if (timeChange >= sampleTime)
	        {
	            /*Compute all the working error variables*/
	            double error = setpoint - input;
	            iTerm += (ki * error);
	            if (iTerm > outMax)
	                iTerm = outMax;
	            else if (iTerm < outMin)
	                iTerm = outMin;
	
	            double dInput = (input - lastInput);
	
	            /*Compute PID Output*/
	            output = (kp * error) + iTerm - (kd * dInput);
	            if (output > outMax)
	                output = outMax;
	            else if (output < outMin)
	                output = outMin;
	
	            /*Remember some variables for next time*/
	            lastInput = input;
	            lastTime = now;
	        }
        }
        
        return output;
    }

    /*
     * setTunings
     * 
     * This updates the proportional, integral and derivative values.
     * 
     * Parameters
     * 	kp			proportional value
     * 	ki			integral value
     * 	kd			derivative value
     *
     * Returns
     * 	nothing.
     */
    public void setTunings(double kp, double ki, double kd)
    {
        if (kp < 0 || ki < 0|| kd < 0)
            return;

        double sampleTimeInSec = ((double)sampleTime)/1000;

        this.kp = kp;
        this.ki = ki * sampleTimeInSec;
        this.kd = kd / sampleTimeInSec;

        if(controllerDirection == TUNING_DIRECTION.REVERSE)
        {
            kp = (0 - kp);
            ki = (0 - ki);
            kd = (0 - kd);
        }
    }

    /*
     * setSampleTime
     * 
     * This sets the sample time in milliseconds used in the compute() method
     * and updates the integral and derivative values.
     * 
     * Parameters
     * 	newSampleTime		number of milliseconds to elapse before computing
     * 						new ouptput.
     * 
     * Returns
     * 	nothing.
     */
    public void setSampleTime(int newSampleTime)
    {
        if (newSampleTime > 0)
        {
            double ratio  = (double)newSampleTime / (double)sampleTime;
            ki *= ratio;
            kd /= ratio;
            sampleTime = (long)newSampleTime;
        }
    }

    /*
     * setOutputLimits
     * 
     * This method sets the minimum and maximum output limits to be returned 
     * from the compute() method.
     * 
     * Parameters
     * 	min			the minimum output value
     * 	max			the maximum output value
     * 
     * Returns
     * 	nothing.
     */
    public void setOutputLimits(double min, double max)
    {
        if (min > max)
            return;

        outMin = min;
        outMax = max;

        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;

        if (iTerm > outMax)
            iTerm = outMax;
        else if (iTerm < outMin)
            iTerm = outMin;
    }

    /*
     * setMode
     * 
     * Sets the operating mode of the tool  If the mode is automatic, the
     * compute() will automatically generate a new output.  If in manual 
     * mode, the compute() method will not generate outputs.  
     * 
     * If switching from manual to automaic, reinitialize internal data.
     * 
     * Parameters
     * 	mode		operating mode, automatic or manual.
     * 
     * Returns
     * 	nothing.
     */
    public void setMode(OPERATING_MODE mode)
    {
        boolean newAuto = (mode == OPERATING_MODE.AUTOMATIC);
        if(newAuto == !inAuto)
        {  /*we just went from manual to auto*/
            reinitialize();
        }
        inAuto = newAuto;
    }

    /*
     * setControllerDirection
     * 
     * Sets the direction of the tuning.
     * 
     * Parameters
     * 	direction	Forward or Reverse.
     * 
     * Returns
     * 	nothing.
     */
    public void setControllerDirection(TUNING_DIRECTION direction)
    {
        controllerDirection = direction;
    }

    /*
     * getKp
     * 
     * Returns the current proportional value
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the current proportional value
     */
	public double getKp()
	{
		return kp;
	}
	
    /*
     * getKi
     * 
     * Returns the current integral value
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the current integral value
     */
	public double getKi()
	{
		return ki;
	}
	
    /*
     * getKd
     * 
     * Returns the current derivative value
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the current derivative value
     */
	public double getKd()
	{
		return kd;
	}
	
    /*
     * setSetpoint
     * 
     * Sets a new target setpoint
     * 
     * Parameters
     * 	setpoint	the new target setpoint.
     * 
     * Returns
     * 	nothing.
     */
	public void setSetpoint(double setpoint)
	{
		this.setpoint = setpoint;
	}
	
    /*
     * getSetpoint
     * 
     * Gets the current target setpoint
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the current setpoint.
     */
	public double getSetpoint()
	{
		return setpoint;
	}
	
    /*
     * setInput
     * 
     * Sets a new input value.
     * 
     * Parameters
     * 	input		the new input value
     * 
     * Returns
     * 	nothing
     */
	public void setInput(double input)
	{
		this.input = input;
	}
	
    /*
     * getInput
     * 
     * Gets the current input value
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the current input value.
     */
	public double getInput()
	{
		return input;
	}
	
    /*
     * getOutput
     * 
     * Gets the last calculated output value
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	the last calculated output value.
     */
	public double getOutput()
	{
		return output;
	}

    /*
     * setOutput
     * 
     * Sets the output value
     * 
     * Parameters
     * 	output		the new output value
     * 
     * Returns
     * 	nothing.
     */
	public void setOutput(double output)
	{
		this.output = output;
	}

	
	/*
	 * PRIVATE METHODS
	 */

	/*
	 * void init()
	 * 
	 * this is a private method that set default values into class data.
	 * 
	 * Parameters
	 * 	none
	 * 
	 * Returns
	 * 	none
	 */
    private void init()
    {
    	
		input = 0;
		output = 0;
		setpoint = 0;
        controllerDirection = TUNING_DIRECTION.DIRECT;
        sampleTime = 100;
        inAuto = false;
		outMin = 0;
		outMax = 255;
        lastTime = System.currentTimeMillis();
    }

    /*
     * reinitialize
     * 
     * reinitialize the internal data after changing operating modes
     * 
     * Parameters
     * 	none
     * 
     * Returns
     * 	nothing.
     */
    private void reinitialize()
    {
        lastInput = input;
        iTerm = output;
        if (iTerm > outMax)
            iTerm= outMax;
        else if (iTerm < outMin)
            iTerm= outMin;
    }

	
}

