package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AutoRotate extends Command {
    Drivetrain m_drivetrain;
    Vision m_vision;
    double m_rotateAngle;
    double currentAngle;
    public AutoRotate (Drivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        addRequirements(drivetrain); // Don't put vision in requirements or else the command will be canceled
    }
    @Override
    public void initialize() {
        if (m_vision.hasCorrectTargets()) {
            currentAngle = Math.atan(m_vision.getXMeters().get()/m_vision.getZMeters().get());
        }
    }

    @Override
    public void execute () {
        m_drivetrain.curvatureDrive(0, MathUtil.clamp(currentAngle, -1, 1), false, true);
    }
    @Override
    public boolean isFinished (){
        if (!m_vision.hasCorrectTargets()) return true;
        if (m_vision.getAngleToTarget().isEmpty()) return false;//TODO: replace with true if consistency is good
        
        return Math.abs(m_vision.getAngleToTarget().get()) < 4.29592040; //very important
    }
}
