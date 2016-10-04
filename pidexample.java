
public class pidexample
{
	static double setpoint;
	static double input;
	static double output;
	
	static double aggKp = 4;
	static double aggKi = 0.2;
	static double aggKd = 1;
	
	static double consKp = 1;
	static double consKi = 0.05;
	static double consKd = 0.25;
	
	static double gyroPos = 0.0;
	static double motorSpeed = 0.0;
	private static PID2903 pid2903;

	public static void main(String[] args)
	{
		// setup
		setpoint = 90;
		input = 0;
		output = 0;
		pid2903 = new PID2903(consKp, consKi, consKd, PID2903.TUNING_DIRECTION.DIRECT);

		// set initial input and output values
		pid2903.setInput(input);
		pid2903.setOutput(output);
		
		// set target value (setpoint)
		pid2903.setSetpoint(setpoint);
		
		// set minimum and maximum output -- for example power to motors
		pid2903.setOutputLimits(0, 100);
		
		// enable the PID
		pid2903.setMode(PID2903.OPERATING_MODE.AUTOMATIC);
		
		int x = 0;
		gyroPos = 0;
		
		do
		{
			loop();
			System.out.format("gyroPos = %f ***** output = %f%n", gyroPos, output);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			gyroPos += Math.random() * 10;
			
		}
		while (gyroPos < setpoint);
	}
	
	static void loop()
	{
		input = gyroPos;
		pid2903.setInput(input);
		
		double gap = setpoint - input;
		if (gap < 10)
		{
			// we're getting close to the goal
			pid2903.setTunings(consKp, consKi, consKd);
		}
		else
		{
			// we're still far from goal, use aggressive values
			pid2903.setTunings(aggKp, aggKi, aggKd);
		}
		
		output = pid2903.compute();
//		output = pid2903.getOutput();
	}
}