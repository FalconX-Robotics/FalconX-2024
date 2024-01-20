package frc.robot.simulation;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class RelativeEncoderSim implements RelativeEncoder {

    double m_offset = 0.0;
    double m_simulationVelocity = 0.0;
    double m_simulationPostion = 0.0;

    @Override
    public double getPosition() {
        return m_simulationPostion - m_offset;
    }

    @Override
    public double getVelocity() {
        return m_simulationVelocity;
    }

    @Override
    public REVLibError setPosition(double position) {
        m_offset = m_simulationPostion - position;
        return REVLibError.kOk;
    }

    public void setSimulationVelocityMetersPerSecond(double velocity) {
        m_simulationVelocity = velocity; 
    }

    public void setSimulationPositionMeters(double position) {
        m_simulationPostion = position;
    }
    
    public void zeroSimulationPostion() {
        m_simulationPostion = 0.0;
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return REVLibError.kOk;
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return REVLibError.kOk;
    }

    @Override
    public double getPositionConversionFactor() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getVelocityConversionFactor() {
        throw new UnsupportedOperationException();
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getAverageDepth() {
        throw new UnsupportedOperationException();
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getMeasurementPeriod() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getCountsPerRevolution() {
        throw new UnsupportedOperationException();
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean getInverted() {
        throw new UnsupportedOperationException();
    }
    
}