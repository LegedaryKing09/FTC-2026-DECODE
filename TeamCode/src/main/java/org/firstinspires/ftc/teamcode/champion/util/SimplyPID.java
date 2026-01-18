package org.firstinspires.ftc.teamcode.champion.util;

public class SimplyPID {
    private double setPoint;
    private double kP, kI, kD;

    private double minLimit = Double.NaN, maxLimit = Double.NaN;

    private double previousTime = Double.NaN;
    private double lastError = 0;
    private double integralError = 0;

    public SimplyPID(double setPoint, double kP, double kI, double kD) {
        this.setSetpoint(setPoint);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getOutput(double currentTime, double currentValue) {
        double error = setPoint - currentValue;

        double dt = (!Double.isNaN(previousTime)) ? (currentTime - previousTime) : 0;

        double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        integralError += error * dt;

        previousTime = currentTime;
        lastError = error;

        return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError));
    }

    public void reset() {
        previousTime = Double.NaN;
        lastError = 0;
        integralError = 0;
    }

    private double checkLimits(double output) {
        if (!Double.isNaN(minLimit) && output < minLimit)
            return minLimit;
        else if (!Double.isNaN(maxLimit) && output > maxLimit)
            return maxLimit;
        else
            return output;
    }

    public void setOuputLimits(double minLimit, double maxLimit) {
        if (minLimit < maxLimit) {
            this.minLimit = minLimit;
            this.maxLimit = maxLimit;
        } else {
            this.minLimit = maxLimit;
            this.maxLimit = minLimit;
        }
    }

    public void removeOuputLimits() {
        minLimit = Double.NaN;
        maxLimit = Double.NaN;
    }

    public void setSetpoint(double setPoint) {
        reset();
        this.setPoint = setPoint;
    }
}

