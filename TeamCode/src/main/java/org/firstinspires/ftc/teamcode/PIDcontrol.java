package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontrol {
    private double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime timer = new ElapsedTime();
    double lastError =0;


    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
    public void SetPIDCoeficients(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}
