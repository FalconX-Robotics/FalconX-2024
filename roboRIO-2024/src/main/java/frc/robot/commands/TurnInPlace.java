package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnInPlace extends Command{
     Drivetrain m_drivetrain;

    public TurnInPlace(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        m_drivetrain.setTurnInPlace(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setTurnInPlace(false);
    }
}
