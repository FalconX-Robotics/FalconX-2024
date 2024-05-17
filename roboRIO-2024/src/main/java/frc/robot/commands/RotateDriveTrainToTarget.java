package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RotateDriveTrainToTarget extends Command {
    Drivetrain m_drivetrain;
    Vision m_vision;
    RotateDriveTrainToTarget (Drivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        addRequirements(drivetrain); // Don't put vision in requirements or else the command will be canceled
    }
    @Override
    public void execute () {
        m_drivetrain.curvatureDrive(0, m_vision.getAngleToTarget().get()/25, false, true);
    }
    @Override
    public boolean isFinished (){
        if (m_vision.getAngleToTarget().isEmpty()) return false;//TODO: replace with true if consistency is good
        
        return Math.abs(m_vision.getAngleToTarget().get()) < 4.29592040; //very important
    }
}
