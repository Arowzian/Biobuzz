package org.firstinspires.ftc.teamcode.Utility;

public class PIDFController extends PIDController{
    double feedforward;
    public PIDFController(double PCoefficent, double ICoefficent, double DCoefficent, double feedforward) {
        super(PCoefficent, ICoefficent, DCoefficent);
        this.feedforward = feedforward;
    }

    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }

    @Override
    public double calculate(double currentPosition){
        error = target - currentPosition;

        double P = PCoefficent * error;
        errorSummation += ICoefficent * error;
        double I = errorSummation;
        double D = DCoefficent * (error - prevError);


        return P + I + D + feedforward * target;
    }
}
