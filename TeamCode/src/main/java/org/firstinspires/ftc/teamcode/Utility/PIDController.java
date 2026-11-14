package org.firstinspires.ftc.teamcode.Utility;

public class PIDController {

    protected double PCoefficent;
    protected double ICoefficent;
    protected double DCoefficent;

    protected double target;
    protected double error;
    protected double prevError;
    protected double errorSummation;

    // Remember that it's good practice to set the target as whatever the starting value is to avoid large power jumps
    public PIDController(double PCoefficent, double ICoefficent, double DCoefficent){
        this.PCoefficent = PCoefficent;
        this.ICoefficent = ICoefficent;
        this.DCoefficent = DCoefficent;
    }

    public void setPCoefficent(double PCoefficent){ this.PCoefficent = PCoefficent; }
    public void setICoefficent(double ICoefficent){ this.ICoefficent = ICoefficent; }
    public void setDCoefficent(double DCoefficent){ this.DCoefficent = DCoefficent; }


    public void setTarget(double target) { this.target = target; }

    public double rotationCalculate(double currentPosition){
        error = HardwareMapsies.angleWrap(target - currentPosition);

        double P = PCoefficent * error;
        errorSummation += ICoefficent * error;
        double I = errorSummation;
        double D = DCoefficent * (error - prevError);

        prevError = error;

        return P + I + D;
    }

    public double calculate(double currentPosition){
        error = target - currentPosition;

        double P = PCoefficent * error;
        errorSummation += ICoefficent * error;
        double I = errorSummation;
        double D = DCoefficent * (error - prevError);

        prevError = error;

        return P + I + D;
    }
}
