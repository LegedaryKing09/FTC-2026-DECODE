/*
 * Copyright (c) 2018 Charles Grassin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package fr.charleslabs.simplypid;


public class SimplyPID {
	// PID coefficients
    private double setPoint;
    private double kP,kI,kD;

    /** Limit bound of the output. */
    private double minLimit = Double.NaN ,maxLimit = Double.NaN;
    
    // Dynamic variables
    private double previousTime = Double.NaN;
    private double lastError = 0;
    private double integralError = 0;

    /**
     * Constructs a new PID with set coefficients.
     *
     * @param setPoint The initial target value.
     * @param kP The proportional gain coefficient.
     * @param kI The integral gain coefficient.
     * @param kD The derivative gain coefficient.
     */
    public SimplyPID(final double setPoint, final double kP, final double kI, final double kD) {
    	this.setSetpoint(setPoint);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getOutput(final double currentTime, final double currentValue) {
    	final double error = setPoint - currentValue;
    	final double dt = (previousTime != Double.NaN) ?
    			(double)(currentTime - previousTime) : 0;
    	
    	// Compute Integral & Derivative error
        final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        integralError += error * dt;
        
        // Save history
        previousTime = currentTime;
        lastError = error;
        
        return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError));
    }

    public void reset() {
        previousTime = 0;
        lastError = 0;
        integralError = 0;
    }

    private double checkLimits(final double output){
    	if (!Double.isNaN(minLimit) && output < minLimit)
    		return minLimit;
    	else if (!Double.isNaN(maxLimit) && output > maxLimit)
    		return maxLimit;
    	else
    		return output;
    }
    
    //Getters & Setters

    public void setOuputLimits(final double minLimit, final double maxLimit) {
    	if(minLimit < maxLimit) {
    		this.minLimit = minLimit;
    		this.maxLimit = maxLimit;
    	}else{
    		this.minLimit = maxLimit;
    		this.maxLimit = minLimit;
    	}
    }
    
    /**
     * Removes the output limits of the PID controller
     *
     * @param
     */
    public void removeOuputLimits() {
        this.minLimit = Double.NaN;
        this.maxLimit = Double.NaN;
    }
    
	/**
	 * @return the kP parameter
	 */
	public double getkP() {
		return kP;
	}

	/**
	 * @param kP the kP parameter to set
	 */
	public void setkP(double kP) {
		this.kP = kP;
        reset();
	}

	/**
	 * @return the kI parameter
	 */
	public double getkI() {
		return kI;
	}

	/**
	 * @param kI the kI parameter to set
	 */
	public void setkI(double kI) {
		this.kI = kI;
        reset();
	}

	/**
	 * @return the kD parameter
	 */
	public double getkD() {
		return kD;
	}

	/**
	 * @param kD the kD parameter to set
	 */
	public void setkD(double kD) {
		this.kD = kD;
        reset();
	}

	/**
	 * @return the setPoint
	 */
	public double getSetPoint() {
		return setPoint;
	}
	
    /**
     * Establishes a new set point for the PID controller.
     *
     * @param setPoint The new target point.
     */
    public void setSetpoint(final double setPoint) {
        reset();
        this.setPoint = setPoint;
    }
}
