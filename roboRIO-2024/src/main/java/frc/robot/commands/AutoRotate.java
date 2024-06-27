package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AutoRotate extends Command {
    Drivetrain m_drivetrain;
    Vision m_vision;
    PigeonIMU m_gyro;
    double m_rotateAngle;
    double distanceToRotate;
    PIDController pid = new PIDController(0.03, 0, 0.005);
    double feedforward = 0.2;
    public AutoRotate (Drivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_gyro = drivetrain.getGyro();
        addRequirements(drivetrain); // Don't put vision in requirements or else the command will be canceled
    }
    @Override
    public void initialize() {
        if (m_vision.hasCorrectTargets()) {
            distanceToRotate = -m_vision.getYaw().get(); //degrees
            // distanceToRotate = Math.atan2(m_rotateAngle, distanceToRotate)
            // distanceToRotate = Math.atan(m_vision.getXMeters().get()/m_vision.getZMeters().get());
        }
        double cameraOffset = 10;
        pid.setSetpoint(m_gyro.getYaw() + distanceToRotate + cameraOffset);
        pid.setTolerance(0.5);
    }

    @Override
    public void execute () {
        DashboardHelper.putNumber(LogLevel.Debug, "Auto Rotate auto rotate angle", distanceToRotate);
        if (m_vision.hasCorrectTargets()) {
            DashboardHelper.putNumber(LogLevel.Debug, "Auto Rotate current distance to rotate", -m_vision.getYaw().get());
            // distanceToRotate = -m_vision.getYaw().get();
            // distanceToRotate = Math.atan2(m_rotateAngle, distanceToRotate)
            // distanceToRotate = Math.atan(m_vision.getXMeters().get()/m_vision.getZMeters().get());
        }
        double rotateSpeed = pid.calculate(m_gyro.getYaw());
        rotateSpeed += feedforward * Math.signum(rotateSpeed);
        rotateSpeed = MathUtil.clamp(rotateSpeed, -1, 1);
        if (pid.atSetpoint()) {rotateSpeed = 0;}
        DashboardHelper.putNumber(LogLevel.Debug, "Auto Rotate Rotate Speed", rotateSpeed);
        DashboardHelper.putNumber(LogLevel.Debug, "Auto Rotate Distance from setpoint", pid.getPositionError());
        // rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.1);
        m_drivetrain.curvatureDrive(0, rotateSpeed, false, true);
    }
    
    @Override
    public boolean isFinished () {
        return false;
        // if (!m_vision.hasCorrectTargets()) return true;
        // return false;
        // if (m_vision.getAngleToTarget().isEmpty()) return false;//TODO: replace with true if consistency is good
        
        // return Math.abs(distanceToRotate) < 0.04; //very important
    }
}
