package com.qualcomm.ftcrobotcontroller.opmodes;

enum OPERATING_MODE
{
    MANUAL,
    AUTOMATIC
}

enum TUNING_DIRECTION
{
    DIRECT,
    REVERSE
}


public class PID2903
{
    // private data
    private long lastTime;
    private double Input, Output, Setpoint;
    private double ITerm, lastInput;
    private double kp, ki, kd;
    private long SampleTime;
    private double outMin, outMax;
    private boolean inAuto;

    private TUNING_DIRECTION controllerDirection;

    public void init()
    {
        controllerDirection = TUNING_DIRECTION.DIRECT;
        SampleTime = 1000;
        inAuto = false;
    }

    public void compute()
    {
        if (!inAuto)
            return;

        long now = System.currentTimeMillis() % 1000;
        long  timeChange = (now - lastTime);
        if (timeChange >= SampleTime)
        {
            /*Compute all the working error variables*/
            double error = Setpoint - Input;
            ITerm += (ki * error);
            if (ITerm > outMax)
                ITerm= outMax;
            else if (ITerm < outMin)
                ITerm= outMin;

            double dInput = (Input - lastInput);

            /*Compute PID Output*/
            Output = kp * error + ITerm- kd * dInput;
            if (Output > outMax)
                Output = outMax;
            else if (Output < outMin)
                Output = outMin;

            /*Remember some variables for next time*/
            lastInput = Input;
            lastTime = now;
        }
    }

    public void SetTunings(double Kp, double Ki, double Kd)
    {
        if (Kp < 0 || Ki < 0|| Kd < 0)
            return;

        double SampleTimeInSec = ((double)SampleTime)/1000;

        kp = Kp;
        ki = Ki * SampleTimeInSec;
        kd = Kd / SampleTimeInSec;

        if(controllerDirection == TUNING_DIRECTION.REVERSE)
        {
            kp = (0 - kp);
            ki = (0 - ki);
            kd = (0 - kd);
        }
    }

    public void SetSampleTime(int NewSampleTime)
    {
        if (NewSampleTime > 0)
        {
            double ratio  = (double)NewSampleTime / (double)SampleTime;
            ki *= ratio;
            kd /= ratio;
            SampleTime = (long)NewSampleTime;
        }
    }

    void SetOutputLimits(double Min, double Max)
    {
        if (Min > Max)
            return;

        outMin = Min;
        outMax = Max;

        if (Output > outMax)
            Output = outMax;
        else if (Output < outMin)
            Output = outMin;

        if (ITerm > outMax)
            ITerm= outMax;
        else if (ITerm < outMin)
            ITerm= outMin;
    }

    void SetMode(OPERATING_MODE Mode)
    {
        boolean newAuto = (Mode == OPERATING_MODE.AUTOMATIC);
        if(newAuto == !inAuto)
        {  /*we just went from manual to auto*/
            ReInitialize();
        }
        inAuto = newAuto;
    }

    void ReInitialize()
    {
        lastInput = Input;
        ITerm = Output;
        if (ITerm > outMax)
            ITerm= outMax;
        else if (ITerm < outMin)
            ITerm= outMin;
    }

    void SetControllerDirection(TUNING_DIRECTION Direction)
    {
        controllerDirection = Direction;
    }

}

