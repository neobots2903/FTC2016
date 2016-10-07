
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
	
		// set target value (setpoint)
		pid2903.setSetpoint(setpoint);
		
		// set minimum and maximum output -- for example power to motors
		pid2903.setOutputLimits(0, 100);
		
		// enable the PID
		pid2903.setMode(PID2903.OPERATING_MODE.AUTOMATIC);
		
		int x = 0;
		gyroPos = 0;

		
//		try {
//			Thread.sleep(100);
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		output = pid2903.compute(input);
		
		do
		{
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			loop();
			System.out.format("gyroPos = %f ***** output = %f%n", gyroPos, output);
			
			if (output > 0)
				gyroPos += Math.random() * 10;
			
		}
		while (gyroPos < setpoint);
		System.out.format("gyroPos = %f ***** output = %f%n", gyroPos, output);
	}
	
	static boolean setCons = false;
	static boolean setAgg = false;
	
	static void loop()
	{
		input = gyroPos;
		
		double gap = setpoint - input;
		if (gap < 10)
		{
			if (!setCons) {
				// we're getting close to the goal
				pid2903.setTunings(consKp, consKi, consKd);
				setCons = true;
			}
		}
		else
		{
			if (!setAgg){
				// we're still far from goal, use aggressive values
				pid2903.setTunings(aggKp, aggKi, aggKd);
				setAgg = true;
			}
		}
		
		output = pid2903.compute(input);
	}
}