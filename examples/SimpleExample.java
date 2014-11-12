import com.matigakis.controlsystems.pid.PID;

/**
 * Example on the usage of the PID controller.
 * 
 * In this example it is demonstrated how the output of the PID controller
 * changes given an error value.
 */
public class SimpleExample {
	public static void main(String[] args) {
		double dt = 0.01;
		double kp = 1.5;
		double ki = 0.01;
		double kd = 0.001;
		
		double minOut = 0.0;
		double maxOut = 1.0;
		
		double maxIntegral = 10.0;
		
		PID pid = new PID(kp, ki, kd, minOut, maxOut, dt, maxIntegral);
		
		double[] errors = {10.0, 5.0, 1.0, 0.5, 0.3, 0.1, 0.01, 0.001, 0.001, 0.001, 0.001};
		
		System.out.println("Time\tError\tOutput");
		
		for(int i = 0; i < errors.length; i++){
			double result = pid.calculateOutput(errors[i]);
			System.out.println(i * dt + "\t" + errors[i] + "\t" + result);	
		}
	}
}
