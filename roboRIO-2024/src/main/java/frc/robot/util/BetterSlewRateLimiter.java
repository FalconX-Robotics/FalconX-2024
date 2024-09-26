package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class BetterSlewRateLimiter {

    double m_accel;
    double m_decel;
    double m_prevVal;
    double m_prevTime;

    public BetterSlewRateLimiter(double accel, double decel, double initialValue) {
        m_accel = accel;
        m_decel = decel;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;

    }

    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        // m_prevVal +=
        //     MathUtil.clamp(
        //         input - m_prevVal,
        //         m_decel * elapsedTime,
        //         m_accel * elapsedTime);

        if (Math.abs(input) > Math.abs(m_prevVal) && Math.signum(input) == Math.signum(m_prevVal)) {
            m_prevVal += MathUtil.clamp(input - m_prevVal,  -m_accel * elapsedTime, m_accel * elapsedTime);
        } else {
            m_prevVal += MathUtil.clamp(input - m_prevVal,  -m_decel * elapsedTime, m_decel * elapsedTime);
        }
        m_prevTime = currentTime;
        return m_prevVal;
      }
}