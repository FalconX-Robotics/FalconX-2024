package frc.robot.commands;

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
    double m_rotateAngle;
    double distanceToRotate;
    PIDController pid = new PIDController(distanceToRotate, m_rotateAngle, distanceToRotate);
    public AutoRotate (Drivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        addRequirements(drivetrain); // Don't put vision in requirements or else the command will be canceled
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute () {
        DashboardHelper.putNumber(LogLevel.Debug, "auto rotate angle", distanceToRotate);
        if (m_vision.hasCorrectTargets()) {
            distanceToRotate = -m_vision.getYaw().get();
            // distanceToRotate = Math.atan2(m_rotateAngle, distanceToRotate)
            // distanceToRotate = Math.atan(m_vision.getXMeters().get()/m_vision.getZMeters().get());
        }
        m_drivetrain.curvatureDrive(0, MathUtil.clamp(distanceToRotate/5, -0.25, 0.25), false, true);
    }
    @Override
    public boolean isFinished (){
        if (!m_vision.hasCorrectTargets()) return true;
        // if (m_vision.getAngleToTarget().isEmpty()) return false;//TODO: replace with true if consistency is good
        
        return Math.abs(distanceToRotate) < 0.1; //very important
    }
}
