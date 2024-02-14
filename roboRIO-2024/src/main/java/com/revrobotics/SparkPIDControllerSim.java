package com.revrobotics;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.SparkPIDController;

public class SparkPIDControllerSim extends SparkPIDController {
    private static class PIDGains {
        double p;
        double i;
        double d;
        double ff;
        double minOutput;
        double maxOutput;
        double iZone;

        PIDGains() {
            p = 0.;
            i = 0.;
            d = 0.;
            ff = 0.;
            iZone = 0.;
            minOutput = -1.;
            maxOutput = 1.;
        }
    }

    PIDGains[] gains = new PIDGains[4];
    double m_setpoint = 0;
    int m_slot = 0;

    public SparkPIDControllerSim(CANSparkBase device) {
        super(device);
        for (int i = 0; i < 4; i++) {
            gains[i] = new PIDGains();
        }
    }

    @Override
    public REVLibError setP(double gain, int slotID) {
        gains[slotID].p = gain;
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setI(double gain, int slotID) {
        gains[slotID].i = gain;
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setD(double gain, int slotID) {
        gains[slotID].d = gain;
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setFF(double gain, int slotID) {
        gains[slotID].ff = gain;
        return REVLibError.kOk;
    }


    @Override
    public REVLibError setOutputRange(double min, double max, int slotID) {
        gains[slotID].minOutput = min;
        gains[slotID].maxOutput = max;
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setIZone(double iZone, int slotID) {
        gains[slotID].iZone = iZone;
        return REVLibError.kOk;
    }


    @Override
    public REVLibError setReference(double value, ControlType ctrl) {
        if (ctrl == ControlType.kVelocity) {
            m_setpoint = value;
            return REVLibError.kOk;
        }

        return REVLibError.kError;
    }

    private double m_percentOutput = 0;
    public double getVoltageOutput() {
        return m_percentOutput * RobotController.getBatteryVoltage();
    }

    private double m_iState = 0.;
    private double m_prevError = 0.;
    private double m_time = 0.;

    private double m_velocity;
    public void setVelocity(double velocity) {
        m_velocity = velocity;
    }

    // From https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
    public void update(double dt) {
        if (DriverStation.isDisabled()) {
            m_time = 0;
            m_prevError = 0;
            m_iState = 0.;
            return;
        }

        dt += m_time;
        while (dt >= 0.001) {
            double error = m_setpoint - m_velocity;
            double p = error * gains[m_slot].p;

            if (Math.abs(error) <= gains[m_slot].iZone || gains[m_slot].iZone == 0.0) {
                m_iState += error * gains[m_slot].i;
            } else {
                m_iState = 0;
            }

            double d = (error - m_prevError) * gains[m_slot].d;
            m_prevError = error;

            double f = m_setpoint * gains[m_slot].ff;

            m_percentOutput = p + m_iState + d + f;
            m_percentOutput = Math.max(
                gains[m_slot].minOutput,
                Math.min(m_percentOutput,
                        gains[m_slot].maxOutput));
            dt -= 0.001;
        }

        m_time = Math.max(0., dt);
    }
}
