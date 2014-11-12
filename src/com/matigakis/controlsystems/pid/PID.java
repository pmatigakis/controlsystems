package com.matigakis.controlsystems.pid;

/**
 * 
 * Implementation of a PID controller
 */
public class PID {
	private double kp;
	private double ki;
	private double kd;
	
	private double dt;
	
	private double maxIntegral;
	private double integral;
	
	private double minOut;
	private double maxOut;
	
	private double last_error;
	
	/**
	 * Create a PID controller
	 * 
	 * @param kp proportional term
	 * @param ki integral term
	 * @param kd derivative term
	 * @param minOut minimum output value
	 * @param maxOut maximum output value
	 * @param dt time constant
	 * @param maxIntegral maximum integration value
	 */
	public PID(double kp, double ki, double kd, double minOut, double maxOut, double dt, double maxIntegral){
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		
		this.minOut = minOut;
		this.maxOut = maxOut;
		
		this.dt = dt;
		
		this.maxIntegral = maxIntegral;
		
		last_error = 0.0;
		integral = 0.0;
	}
	
	/**
	 * Get the proportional term
	 * 
	 * @return the proportional term
	 */
	public double getKP(){
		return kp;
	}

	/**
	 * Get the integral term
	 * 
	 * @return the integral term
	 */
	public double getKI(){
		return kp;
	}

	/**
	 * Get the derivative term
	 * 
	 * @return the derivative term
	 */
	public double getKD(){
		return kp;
	}

	/**
	 * Get the time constant
	 * 
	 * @return
	 */
	public double getDT(){
		return dt;
	}
	
	/**
	 * Get the maximum integration value
	 * 
	 * @return
	 */
	public double getMaxIntegral(){
		return maxIntegral;
	}
	
	/**
	 * Get the minimum output value
	 * 
	 * @return
	 */
	public double getMinOutput(){
		return minOut;
	}

	/**
	 * Get the maximum output value
	 * 
	 * @return
	 */
	public double getMaxOutput(){
		return maxOut;
	}
	
	/**
	 * Change the kp term
	 * 
	 * @param kp
	 */
	public void setKP(double kp){
		this.kp = kp;
	}

	/**
	 * Change the ki term
	 * 
	 * @param ki
	 */
	public void setKI(double ki){
		this.ki = ki;
	}

	/**
	 * Change the kd term
	 * 
	 * @param kd
	 */
	public void setKD(double kd){
		this.kd = kd;
	}
	
	/**
	 * calculate the PID controller output
	 * 
	 * @param error the current error
	 * @return PID controller output
	 */
	public double calculateOutput(double error){
		integral += error * dt;
		
		if(integral > maxIntegral){
			integral = maxIntegral;
		}else if(integral < -maxIntegral){
			integral = -maxIntegral;
		}
		
		double derivative = (error - last_error) / dt;
		
		double output = kp * error + ki * integral + kd * derivative;
	
		if(output < minOut){
			output = minOut;
		}else if(output > maxOut){
			output = maxOut;
		}
		
		last_error = error;
	
		return output;
	}
}
