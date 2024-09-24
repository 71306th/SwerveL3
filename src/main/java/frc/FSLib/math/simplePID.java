package frc.FSLib.math;

public class simplePID {
    private double lastError = 0, d = 0;
    double output;
    private double kP, kD;

    public simplePID(double ikP, double ikD){
        kP = ikP;
        kD = ikD;
    }

    public double calculate (double currentValue, double targetValue){
        double error = currentValue - targetValue;
        d = error - lastError;
        lastError = error;
        output = (error * kP) + (d * kD);
        return output;
    }
}
